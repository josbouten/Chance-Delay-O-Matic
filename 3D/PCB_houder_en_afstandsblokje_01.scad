$fn = 160;




module houderVoorPcb() {
    xdim = 11;
    ydim = 8;
    zdim = 14;
    difference() {
        cube([xdim, ydim, zdim]);
        translate([xdim / 2, 8, 10.5])rotate([90, 0, 0]) cylinder(8, 3.5 / 2, 3.5 / 2);
        translate([4 * xdim / 5, 8/2, 0]) cylinder(14, 3.5 / 2, 3.5 / 2);
        uitsparingx = xdim * 2 / 5;
        transx = xdim * 3 / 5;
        #translate([transx, 0, zdim - 1.5]) cube([uitsparingx, ydim, 1.5]);
    }
}

module afstandsBlokje() {
    xdim = 7;
    ydim = 7;
    zdim = 9.5;
    difference() {
        cube([xdim, ydim, zdim]);
        #translate([xdim / 2, ydim / 2, 0]) cylinder(zdim, 3.5 / 2, 3.5 / 2);
    }
}

module alles(a = 1, b = 1) {
    if (a == 1) {
        houderVoorPcb();
    }
    if (b == 1) {
        translate([20, 0, 0]) afstandsBlokje();
    }
}

alles(1, 1);
        