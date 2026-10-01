// Simplified KY-023 dual-axis joystick module for KiCad 3D visualization.
// Origin: center of the 34 x 26 mm module PCB, Z=0 at host-PCB top.

$fn = 48;
eps = 0.02;

board_x = 34.0;
board_y = 26.0;
board_z = 1.6;
board_bottom = 3.0;
corner_r = 1.0;

hole_dx = 26.50;
hole_dy = 20.75;
hole_d = 3.40;

stick_x = 3.5;
stick_y = 0.0;

module rounded_board_2d() {
    offset(r = corner_r)
        square([board_x - 2 * corner_r, board_y - 2 * corner_r], center = true);
}

module mounting_holes(height) {
    for (x = [-hole_dx / 2, hole_dx / 2])
        for (y = [-hole_dy / 2, hole_dy / 2])
            translate([x, y, -eps]) cylinder(d = hole_d, h = height + 2 * eps);
}

module module_pcb() {
    difference() {
        translate([0, 0, board_bottom])
            linear_extrude(height = board_z) rounded_board_2d();
        mounting_holes(board_bottom + board_z + eps);
    }
}

module mounting_spacers() {
    difference() {
        union()
            for (x = [-hole_dx / 2, hole_dx / 2])
                for (y = [-hole_dy / 2, hole_dy / 2])
                    translate([x, y, 0]) cylinder(d = 5.6, h = board_bottom);
        mounting_holes(board_bottom + eps);
    }
}

module joystick_mechanism() {
    mech_z = board_bottom + board_z;

    // Metal joystick frame and corner retention tabs.
    translate([stick_x, stick_y, mech_z])
        cube([16.5, 16.5, 5.8], center = true);
    for (x = [-7.7, 7.7])
        for (y = [-7.7, 7.7])
            translate([stick_x + x, stick_y + y, mech_z + 0.7])
                cylinder(d = 2.2, h = 2.2);

    // The two orthogonal potentiometer bodies.
    translate([stick_x + 10.0, stick_y, mech_z + 2.3])
        cube([4.0, 13.5, 7.0], center = true);
    translate([stick_x, stick_y + 10.0, mech_z + 2.3])
        cube([13.5, 4.0, 7.0], center = true);

    // Push-button/shaft and recognizable thumb cap.
    translate([stick_x, stick_y, mech_z + 2.0])
        cylinder(d = 6.0, h = 13.0);
    translate([stick_x, stick_y, mech_z + 14.0])
        cylinder(d1 = 11.5, d2 = 10.0, h = 6.0);
    translate([stick_x, stick_y, mech_z + 20.0])
        scale([1, 1, 0.35]) sphere(d = 10.0);
}

module five_pin_header() {
    header_x = -15.75;
    header_y0 = -4.47;
    mech_z = board_bottom + board_z;

    translate([header_x, 0.5, mech_z + 1.25])
        cube([2.5, 12.5, 2.5], center = true);
    for (i = [0 : 4])
        translate([header_x, header_y0 + i * 2.54, board_bottom - 2.0])
            cube([0.65, 0.65, 8.0], center = false);
}

module ky023_module() {
    union() {
        mounting_spacers();
        module_pcb();
        joystick_mechanism();
        five_pin_header();
    }
}

ky023_module();

