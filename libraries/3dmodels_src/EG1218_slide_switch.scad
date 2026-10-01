// E-Switch EG1218 compact SPDT slide switch.
// Origin: center electrical pin (COM), Z=0 at host-PCB top.

$fn = 32;

module eg1218() {
    // Three 0.6 mm square solder pins on 2.50 mm pitch.
    for (x = [-2.5, 0, 2.5])
        translate([x - 0.30, -0.30, -2.8]) cube([0.60, 0.60, 4.0]);

    // Metal shell/body within the footprint's 11.6 x 4.0 mm outline.
    translate([-5.75, -1.95, 1.0]) cube([11.50, 3.90, 2.85]);

    // Black sliding actuator, shown in one stable end position.
    translate([-3.25, -1.10, 3.75]) cube([3.5, 2.20, 1.25]);
}

eg1218();
