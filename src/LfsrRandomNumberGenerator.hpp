#ifndef _RANDOM
#define _RANDOM

#include <stdint.h>

/* 
    Specifying the nrOfBits makes generating the random numbers faster.
    See below to find some statistics.
*/


// 0 = random
// variation of -1 ... 1 
// variation of -2 ... 2

class LFSR_RandomNumberGenerator {

    private:

        uint32_t lfsr;
        uint32_t bit;
        uint32_t randomNumber;

        uint32_t getBit() {
            bit = ((lfsr >> 0) ^ (lfsr >> 10) ^ (lfsr >> 30) ^ (lfsr >> 31)) & 1; // compute the feedback bit
            lfsr = (lfsr >> 1) | (bit << 31); // rotate the register and insert the feedback bit
            return(lfsr & 1);
        }

        uint32_t rand() {
            randomNumber = 0;
            for (int i = 0; i < 32; i++) {
                randomNumber <<= 1;
                randomNumber |= getBit();
            }
            return(randomNumber);
        }

        uint32_t rand(int nrOfBits) {
            randomNumber = 0;
            for (int i = 0; i < nrOfBits; i++) {
                randomNumber <<= 1;
                randomNumber |= getBit();
            }
            return(randomNumber);
        }

        uint32_t random(uint32_t upperLimit) {
            if (upperLimit == 0) {
                return 0 ;
            }
            return rand() % upperLimit;
        }

        uint32_t random(uint32_t upperLimit, int nrOfBits) {
            if (upperLimit == 0) {
                return 0 ;
            }
            return rand(nrOfBits) % upperLimit;
        }
        
        void init(long seed) {
            lfsr = 0x12345678u + seed; // initial state
            bit = 0;
            randomNumber = 0;
        }

    public:
        LFSR_RandomNumberGenerator() {
            init(0);
        }

                // Generate a random number from lowerLimit until and excluding upperLimit
        int getRandomNumber(int lowerLimit, int upperLimit) {
            if (lowerLimit >= upperLimit) {
                return lowerLimit;
            }
            int diff = upperLimit - lowerLimit;
            return random(diff) + lowerLimit;
        }

        /* 
            This method allows to restrict the number of bits used to generate the random number
            and will make it possible for lower numbers to be generated in a shorter 
            amount of time than when using 32 bits.
            
            Here are some statistics for 100.000 numbers:
            32 bits: 1567 mS -> 15.67 micro seconds per calculation
            7 bits :  309 mS ->  3.09 miro seconds per calculation
        */
        int getRandomNumber(int lowerLimit, int upperLimit, int nrOfBits) {
            if (lowerLimit >= upperLimit) {
                return lowerLimit;
            }
            int diff = upperLimit - lowerLimit;
            return random(diff, nrOfBits) + lowerLimit;
        }

};

#endif