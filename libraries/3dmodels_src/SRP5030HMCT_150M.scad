// Bourns SRP5030HMCT-150M shielded power inductor.
// Datasheet nominal body: 5.3 x 5.7 x 2.8 mm.
// Origin: footprint center, Z=0 at host-PCB top.

$fn = 40;

module rounded_prism(size_x, size_y, height, radius) {
    linear_extrude(height = height)
        offset(r = radius)
            square([size_x - 2 * radius, size_y - 2 * radius], center = true);
}

module srp5030() {
    // Thin terminal lands remain visible at the two pad ends.
    translate([-1.75, -2.82, 0.05]) cube([3.50, 1.18, 0.30]);
    translate([-1.75,  1.64, 0.05]) cube([3.50, 1.18, 0.30]);

    // Molded shielded body.
    translate([0, 0, 0.28]) rounded_prism(5.3, 5.7, 2.52, 0.42);
}

srp5030();
