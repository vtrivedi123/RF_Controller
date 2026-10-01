// Simplified 0.96-inch SSD1306 I2C OLED module for KiCad 3D visualization.
// Origin: footprint pad 1 (GND), Z=0 at host-PCB top.

$fn = 48;
eps = 0.02;

board_min_x = -9.69;
board_min_y = -1.50;
board_x = 27.0;
board_y = 27.0;
board_bottom = 2.50;
board_z = 1.60;
corner_r = 0.8;

hole_positions = [
    [-7.89, 0.30],
    [15.51, 0.30],
    [-7.89, 23.70],
    [15.51, 23.70]
];
hole_d = 2.20;

module rounded_board_2d() {
    translate([board_min_x + board_x / 2, board_min_y + board_y / 2])
        offset(r = corner_r)
            square([board_x - 2 * corner_r, board_y - 2 * corner_r], center = true);
}

module oled_board() {
    difference() {
        translate([0, 0, board_bottom])
            linear_extrude(height = board_z) rounded_board_2d();
        for (p = hole_positions)
            translate([p[0], p[1], board_bottom - eps])
                cylinder(d = hole_d, h = board_z + 2 * eps);
    }
}

module pin_header() {
    // Four 2.54-mm pins and their plastic carrier.
    translate([3.81, 0, board_bottom - 0.1])
        cube([10.2, 2.5, 2.5], center = true);
    for (x = [0, 2.54, 5.08, 7.62])
        translate([x - 0.32, -0.32, 0]) cube([0.64, 0.64, board_bottom + board_z + 1.0]);
}

module oled_panel() {
    panel_z = board_bottom + board_z;
    // Glass carrier and raised active display window.
    translate([(board_min_x + 17.31) / 2, (2.50 + 21.76) / 2, panel_z + 0.65])
        cube([26.70, 19.26, 1.30], center = true);
    translate([(-7.06 + 14.68) / 2, (6.55 + 17.75) / 2, panel_z + 1.36])
        cube([21.74, 11.20, 0.18], center = true);
}

module oled_module() {
    union() {
        oled_board();
        pin_header();
        oled_panel();
    }
}

oled_module();

