/*

button cap voor druktoets van Chance-Delay-O-Matic
*/

$fn = 160;

binnendiameter = 3.4;
binnenhoogte = 3;

module binnenCylinder() {
    cylinder(binnenhoogte, r1 = binnendiameter / 2, r2 = binnendiameter / 2);
}

buttonCapHoogte = 9;

buitendiameter = binnendiameter + 2;

module buitenCylinder() {
    cylinder(binnenhoogte, r1 = buitendiameter / 2, r2 = buitendiameter / 2);
}

module buttonCap() {
    cylinder(buttonCapHoogte, r1 = (buitendiameter + 7) * 1 / 6, r2 = buitendiameter * 2 / 6);
}

module alles(a = 0, b = 0) {
    if (a == 1) {
        translate([0, 0, binnenhoogte]) buttonCap();
    }
    if (b == 1) {
        difference() {
            buitenCylinder();
            #binnenCylinder();
        }
    }
}

alles(1, 1);
