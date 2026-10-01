// Standard 15 x 29 mm nRF24L01 breakout (PCB antenna version).
// Origin: footprint pad 1. The 2x4 connector stays registered to U3 pads 1-8,
// while the module body extends toward the controller PCB interior.

$fn = 28;

module module_pcb() {
    translate([-25.5, -2.0, 3.0]) cube([29.0, 15.25, 1.0]);
}

module header() {
    translate([-0.95, -0.95, 0.0]) cube([4.44, 9.52, 3.0]);
    for (x = [0, 2.54])
        for (y = [0, 2.54, 5.08, 7.62])
            translate([x - 0.32, y - 0.32, -1.6]) cube([0.64, 0.64, 5.6]);
}

module radio_components() {
    // Nordic QFN, crystal and representative passives.
    translate([-9.0, 3.0, 4.0]) cube([4.1, 4.1, 0.75]);
    translate([-14.5, 3.5, 4.0]) cube([4.5, 2.0, 0.75]);
    for (p = [[-6.0, 8.6], [-8.5, 9.2], [-11.5, 8.5], [-14.0, 9.5]])
        translate([p[0], p[1], 4.0]) cube([1.6, 0.8, 0.55]);

    // Raised copper-like meander antenna at the far end.
    for (i = [0 : 4])
        translate([-24.5 + i * 1.7, (i % 2) ? 7.0 : 1.0, 4.03])
            cube([1.05, 6.0, 0.22]);
    translate([-24.5, 1.0, 4.03]) cube([8.0, 0.85, 0.22]);
    translate([-24.5, 12.0, 4.03]) cube([8.0, 0.85, 0.22]);
}

union() {
    module_pcb();
    header();
    radio_components();
}
