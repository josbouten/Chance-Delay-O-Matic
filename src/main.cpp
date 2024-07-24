#include <Arduino.h>

/*
  Chance-Delay-O-Matic

  This module will copy gate signals from its input to its output and either change their length, delay them or only copy them by chance.

  It has 2 operating modes. In continuous mode the delay time and gate length can be set on a continuous scale,
  very short (a few milliseconds) and very long (20 seconds) delays are possible. In discrete mode (default mode) the incoming
  pulse is regarded as a clock and the values are quantised to that clock. O.a. this makes it easy to set a delay that is related
  to the speed of the clock/gate received. The code for this module was written (in May 2024) and the hardware was designed (in Nov 2023) by myself.

  Jos Bouten aka ZaphodB, 2024

  Version 0.1
*/


#define VERSION_STR "Chance-Delay-O-Matic v0.1"
//#define DEBUG
#define ATTENTION_SEEKER

#include "Debug.hpp"
#include "OneButton.h"
#include "LfsrRandomNumberGenerator.hpp"


#define EXT_TRIGGER_IN    2 // D2/INT0 interrupt input for extern trigger signal.
#define EXT_TRIGGER_IN_CP 3 // D3/INT1 interrupt input for extern trigger signal.
#define TRIGGER_IN_LED    4 // D4: led to acknowledge extern trigger signal.
#define GATE_OUT_LED      5 // F5: led to signal logic state of trigger output
#define AMP_OUT           6 // D6: AMP_OUT => TRIGGER_OUT because of Q1, TRIGGER_OUT is ! AMP_OUT
#define MANUAL_TRIGGER_IN 7 // D7: hand operated trigger
#define LENGTH_SPEED_SW   8 // D8: switch between low and high speed
#define DELAY_SPEED_SW    9 // D9: switch between short and long delay
#define MODE_LED         10 // D10: shows discrete or analog mode

#define CHANCE_POT       A5
#define DELAY_POT        A4
#define STEP_LENGTH_POT  A3
#define CHANCE_CV        A2
#define DELAY_CV         A1
#define STEP_LENGTH_CV   A0

#define MIN_DELAY   500 // micro seconds
#define MAX_CHANCE  95

#define FAST_DELAY_TIME_RANGE_MIN        500L // Half a milli second using micro seconds as a unit.
#define FAST_DELAY_TIME_RANGE_MAX    2000000L // Two seconds.

#define SLOW_DELAY_TIME_RANGE_MIN    2000000L // Two seconds.
#define SLOW_DELAY_TIME_RANGE_MAX   30000000L // Thirty seconds.

#define FAST_PULSE_LENGTH_RANGE_MIN      500L // Half a milli second.
#define FAST_PULSE_LENGTH_RANGE_MAX  2000000L // Two seconds.

#define SLOW_PULSE_LENGTH_RANGE_MIN  2000000L // Two seconds.
#define SLOW_PULSE_LENGTH_RANGE_MAX 30000000L // Thirty seconds.

#define ATTENTIONSEEKER_TIMEOUT     60000000L // 60.000.000 micro seconds = 60 seconds

#define MIN_STEP_LENGTH 10

OneButton triggerButton = OneButton(MANUAL_TRIGGER_IN, true /* ACTIVE_LOW */, true /* USE_PULLUP */);
LFSR_RandomNumberGenerator randomGenerator;

#define NR_OF_MULT_FACTORS 17
#define T float(2.0/3.0)
#define S float(3.0/2.0)
//                                         0       1        2         3         4        5       6        7      8
float baseFactor[NR_OF_MULT_FACTORS] = { 0.0,   T/32.0, 1.0/32.0, T/16.0, 1.0/16.0, T/8.0, S/16.0, 1.0/8.0, T/4.0,
                                       S/8.0,  1.0/4.0,    T/2.0,  S/4.0,  1.0/2.0,     T,  S/2.0,     1.0 };

#define DISCRETE HIGH
#define ANALOG   LOW
byte mode = DISCRETE;
float minDelayFactor;
float maxDelayFactor;
float minGateDurationFactor;
float maxGateDurationFactor;

bool lengthIsFast() {
  return( ! digitalRead(LENGTH_SPEED_SW) );
}

bool delayIsFast() {
  return( ! digitalRead(DELAY_SPEED_SW) );
}

int stepLengthCv;
int randomVal;
int delayCv;
int chance;
unsigned long gateDuration;
unsigned long gateDurationP;
unsigned long delayTime = MIN_DELAY;
unsigned long delayTimeP;

// Vars used in ISR
volatile unsigned long pulseOnTime= 0L;
volatile unsigned long extClockCycleTime = micros();
volatile unsigned long oldCycleTimeRisingTime = micros();
volatile unsigned long oldRisingTime = micros();
volatile unsigned long thisFallingTime = 0L;
volatile unsigned long thisRisingTime = micros();
volatile byte state = 0;
// The vars allowDelayToContinue and allowGateToContinue are used to stop a delay or a gate
// whenever a shorter delay or gate duration is chosen using one of the switches.
volatile bool allowDelayToContinue = true;
volatile bool allowGateToContinue = true;
//
unsigned long startTime;
unsigned long now;

bool delayIsFastBool = false;
bool oldDelayIsFastBool = false;
bool lengthIsFastBool = false;
bool oldLengthIsFastBool = false;
float multiplicationFactor;
volatile unsigned long now2, startTime2;
volatile bool busy = false;
volatile unsigned long attentionSeekerTimeout;

void manualTrigger() {
  // In case we press the button during a long gate, we want it to go low,
  // even if this is only for a short time.
  // We want to restart the gate.
  // Generate a gate
  digitalWrite(AMP_OUT, HIGH);
  digitalWrite(AMP_OUT, LOW);
  digitalWrite(GATE_OUT_LED, HIGH);
  noInterrupts();
  now2 = startTime2 = micros();
  interrupts();
  while ((now2 - startTime2) < gateDuration) {
    now2 = micros();
  }
  // Stop a running delay.
  allowDelayToContinue = false;
  // Stop a running gate.
  allowGateToContinue = false;
  oldRisingTime = thisRisingTime;
  state = 1;
}

void switchModes() {
  debug_print("Button was double clicked\n");
  // When the button is double clicked, we switch modes
  if (mode == DISCRETE) {
    mode = ANALOG;
  } else {
    mode = DISCRETE;
  }
  digitalWrite(MODE_LED, mode);
  debug_print2("mode: %s\n", mode == DISCRETE? "discrete": "analog");
}

void triggerIsrFalling() {
  digitalWrite(TRIGGER_IN_LED, LOW);
  debug_print("-");
  // We measure the pulse width in microseconds from the rising edge of the
  // incoming pulse to the falling time of that pulse.
  thisFallingTime = micros();
  pulseOnTime = 0.9 * (thisFallingTime - thisRisingTime);
  triggerButton.tick();
  delayIsFastBool = ! digitalRead(DELAY_SPEED_SW);
  // If we toggle the delay fast/slow switch
  // we force the delay timing loop running in loop() to stop.
  if (delayIsFastBool != oldDelayIsFastBool) {
    oldDelayIsFastBool = delayIsFastBool;
    allowDelayToContinue = false;
  }
}

void triggerIsrRising() {
  // We accept both a trigger in pulse and a manual trigger as a trigger.
  debug_print("+");
  thisRisingTime = micros();
  attentionSeekerTimeout = thisRisingTime;
  // We determine the gate duration of the pulse prior to the current one.
  extClockCycleTime = thisRisingTime - oldCycleTimeRisingTime;
  // ToDo: compute the cycle time by taking the mean of several input pulses?
  oldCycleTimeRisingTime = thisRisingTime;
  digitalWrite(TRIGGER_IN_LED, HIGH); // ToDo: find out: how much time does this take? A: a few micro seconds, see readme.jsb.txt
  lengthIsFastBool = ! digitalRead(LENGTH_SPEED_SW);
  if (lengthIsFastBool != oldLengthIsFastBool) {
    oldLengthIsFastBool = lengthIsFastBool;
    allowGateToContinue = false;
  }
  if (!busy) { // busy = true whenever loop() is in the delay loop preceding the gate or in the gate timing loop.
    if ((thisRisingTime - oldRisingTime) < gateDuration) {
      // wait
      debug_print("w");
      state = 0;
      return;
    } else {
        oldRisingTime = thisRisingTime;
        state = 1;
      return;
    }
  }
}

/*
  Fast and slow ranges for delay time and pulse length;
  Fast range is 0.5mS to 2 seconds.
  Slow range is 2 seconds to 30 seconds.
*/

void flashLeds(byte pin1, byte pin2, byte pin3) {
  digitalWrite(pin1, HIGH);
  delay(100);
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  delay(100);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, HIGH);
  delay(100);
  digitalWrite(pin3, LOW);
  digitalWrite(pin2, HIGH);
  delay(100);
  digitalWrite(pin2, LOW);
  digitalWrite(pin1, HIGH);
  delay(100);
}

unsigned long timePast;

void loop() {

  triggerButton.tick();
  switch(state) {
    case 0:
      // If there have been no input clock signals for a while, then
      // show some pseudo activity by flashing the LEDs.
      #ifdef ATTENTION_SEEKER
      noInterrupts();
      // Note: we compute the time that has past since the last clock signal
      // and actively exclude any interrupts to prevent those from interfering.
      timePast = micros() - attentionSeekerTimeout;
      interrupts();
      if (timePast > ATTENTIONSEEKER_TIMEOUT) {
        debug_print("a");
        flashLeds(MODE_LED, TRIGGER_IN_LED, GATE_OUT_LED);
        flashLeds(MODE_LED, TRIGGER_IN_LED, GATE_OUT_LED);
        digitalWrite(MODE_LED, mode);
        attentionSeekerTimeout = micros();
      }
      #endif
      break;
    case 1:
      state = 0;
      if (chance > MAX_CHANCE) chance = 100;
      // We assume that either the pot is used or the CV. If both are used, then the delayCV will hit a ceiling value.
      if (delayCv > 1023) delayCv = 1023;
      randomVal = randomGenerator.getRandomNumber(0, 101, 7);
      if (randomVal <= chance) {
        // We passed the gate end time, so we can fill a new gate time.
        // We assume that either the pot is used or the CV. if both are used, then the stepLength will hit a ceiling value.
        if (mode == ANALOG) {
          if (lengthIsFastBool) {
            gateDuration = map(stepLengthCv, 0, 1023L, FAST_PULSE_LENGTH_RANGE_MIN, FAST_PULSE_LENGTH_RANGE_MAX);
          } else {
            gateDuration = map(stepLengthCv, 0, 1023L, SLOW_PULSE_LENGTH_RANGE_MIN, SLOW_PULSE_LENGTH_RANGE_MAX);
          }
          if ( delayIsFastBool ) {
            delayTime = map(delayCv, 0, 1023, FAST_DELAY_TIME_RANGE_MIN, FAST_DELAY_TIME_RANGE_MAX);
          } else {
            delayTime = map(delayCv, 0, 1023, SLOW_DELAY_TIME_RANGE_MIN, SLOW_DELAY_TIME_RANGE_MAX);
          }
        } else { // We are in DISCRETE mode.
          if (lengthIsFast()) {
            // Short gate duration.
            minGateDurationFactor = NR_OF_MULT_FACTORS;
            maxGateDurationFactor = 12 * NR_OF_MULT_FACTORS; // 12 times the pulse on time
          } else {
            // Extra long gate duration.
            minGateDurationFactor = 12 * NR_OF_MULT_FACTORS; // 12 up to
            maxGateDurationFactor = 36 * NR_OF_MULT_FACTORS; // 36 times the pulse on time.
          }
          gateDurationP = map(stepLengthCv, 0, 1023, minGateDurationFactor, maxGateDurationFactor);
          multiplicationFactor = float(int(gateDurationP / NR_OF_MULT_FACTORS)) + baseFactor[gateDurationP % NR_OF_MULT_FACTORS];
          // We compute a gate out duration as a fraction of multiple of the gate-in signal's ON-duration.
          gateDuration = pulseOnTime * multiplicationFactor;
          //  debug_print6("stepLengthCv: %3d, gateDurationTimeP: %2ld, mult factor: %f, baseFactor[%2ld]: gateDuration: %ld\n",
          //      stepLengthCv, gateDurationP, double(multiplicationFactor), gateDurationP % NR_OF_MULT_FACTORS, gateDuration);

          if ( delayIsFast() ) {
            // Restrict the delay to less than the clock cycle time.
            // If e.g. the clock is a looped gate then this makes it easy to generate in time a new gate within the timing of that loop.
            minDelayFactor = 0;
            maxDelayFactor = 0.9;
          } else {
            minDelayFactor = 0.9; // Allow multiple clock cycle times as gate delay time.
            maxDelayFactor = 12;
          }
          // Quantize the delayTime so that it is a musical 'multiple' of the extClockCycleTime.
          delayTimeP = map(delayCv, 0, 1023, NR_OF_MULT_FACTORS * minDelayFactor, NR_OF_MULT_FACTORS * maxDelayFactor);
          multiplicationFactor = float(int(delayTimeP / NR_OF_MULT_FACTORS)) + baseFactor[delayTimeP % NR_OF_MULT_FACTORS];
          // We compute a delay time as a fraction or multiple of the cycle time of the trigger-in signal.
          delayTime = extClockCycleTime * multiplicationFactor;
          // debug_print6("delayCv: %3d, delayTimeP: %2ld, mult factor: %f, baseFactor[%2ld]: delayTime: %ld\n",
          //     delayCv, delayTimeP, double(multiplicationFactor), delayTimeP % NR_OF_MULT_FACTORS, delayTime);
        }
        // Now compute when to switch the output off.
        triggerButton.tick();
        // We can delay the output signal here.
        busy = true; // Used to block a retrigger when there is a rising edge of the clock during the delay loop.
        if (delayTime > 0) { // pre delay
          now = startTime = micros();
          // Timing loop (in micro second steps) to delay after output has gone HIGH.
          while (((now - startTime) < delayTime) & (allowDelayToContinue == true)) {
            triggerButton.tick();
            now = micros();
          }
          allowDelayToContinue = true;
        }
        digitalWrite(AMP_OUT, LOW);  // AMP_OUT = low therefore Trigger Out = high.
        digitalWrite(GATE_OUT_LED, HIGH);
        triggerButton.tick();
        // Timing loop (in micro second steps) to generate a gate of a certain duration before switching the output LOW again.
        now = startTime = micros();
        while (((now - startTime) < gateDuration) & (allowGateToContinue == true)) {
          triggerButton.tick();
          now = micros();
        }
        allowGateToContinue = true;
        // Set output low again.
        digitalWrite(AMP_OUT, HIGH); // AMP_OUT = high therefore Trigger Out = low.
        digitalWrite(GATE_OUT_LED, LOW);
        busy = false;
      }
      // Read analog values now, so they influence the gate delay the least.
      chance = max(analogRead(CHANCE_POT), analogRead(CHANCE_CV)) / 10.23;
      delayCv = analogRead(DELAY_POT) + analogRead(DELAY_CV);
      stepLengthCv = analogRead(STEP_LENGTH_POT) + analogRead(STEP_LENGTH_CV); // 0 ... 2046
      if (stepLengthCv > 1023) stepLengthCv = 1023;
      break;
    default:
      debug_print2("state: %d should not occur !", state);
      break;
  }
}

void setup() {
  debug_begin(230400);
  debug_print2("\n\nThis is %s\n\n", VERSION_STR);
  debug_print("Begin of setup.\n");
  pinMode(AMP_OUT, OUTPUT);
  pinMode(GATE_OUT_LED, OUTPUT);
  pinMode(EXT_TRIGGER_IN, INPUT_PULLUP);
  pinMode(EXT_TRIGGER_IN_CP, INPUT_PULLUP);
  pinMode(TRIGGER_IN_LED, OUTPUT);
  pinMode(CHANCE_POT, INPUT);
  pinMode(CHANCE_CV, INPUT);
  pinMode(DELAY_POT, INPUT);
  pinMode(DELAY_CV, INPUT);
  pinMode(STEP_LENGTH_POT, INPUT);
  pinMode(STEP_LENGTH_CV, INPUT);
  pinMode(LENGTH_SPEED_SW, INPUT_PULLUP); // Switch between low and high speed.
  pinMode(DELAY_SPEED_SW, INPUT_PULLUP);  // Switch between short and long delay.
  pinMode(LED_BUILTIN, OUTPUT);
  // Define and set the mode led.
  pinMode(MODE_LED, OUTPUT);

  flashLeds(MODE_LED, TRIGGER_IN_LED, GATE_OUT_LED);

  // Attach ISRs but prevent them from starting until we reach the
  // end of the setup routine.
  noInterrupts();
  debug_print("Attaching RISING  interrupt");
  attachInterrupt(digitalPinToInterrupt(EXT_TRIGGER_IN), triggerIsrRising, RISING);
  debug_print(" finished.\n");
  debug_print("Attaching FALLING interrupt");
  attachInterrupt(digitalPinToInterrupt(EXT_TRIGGER_IN_CP), triggerIsrFalling, FALLING);
  debug_print(" finished.\n");
  debug_print("Attaching button click");
  triggerButton.attachClick(manualTrigger);
  triggerButton.attachLongPressStart(switchModes); // .attachDoubleClick(switchModes);
  debug_print(" finished.\n");
  randomGenerator = LFSR_RandomNumberGenerator();

  // Read all pots and switches to get initial values.
  //
  chance = max(analogRead(CHANCE_POT), analogRead(CHANCE_CV)) / 10.23;
  delayCv = analogRead(DELAY_POT) + analogRead(DELAY_CV);
  stepLengthCv = analogRead(STEP_LENGTH_POT) + analogRead(STEP_LENGTH_CV); // 0 ... 2046
  lengthIsFastBool = ! digitalRead(LENGTH_SPEED_SW);
  delayIsFastBool = ! digitalRead(DELAY_SPEED_SW);
  gateDuration = 0L;
  attentionSeekerTimeout = micros();
  debug_print("Setup finished.\n");
  debug_print("\n\nNow connect a clock to make this thing do its thing !\n\n");
  interrupts();
}