# Lab 7 — Part 2: PCB-design AI context

Attach this file with **your schematic, PCB, libraries, 3D models, and any DRC output**. Ask the AI to inspect the actual files, not just a screenshot. Before routing, it can propose a placement sequence or critique your tentative positions using the real outline, footprints, mechanical constraints, and component layout guidance. The schematic should drive the PCB's nets. ERC/DRC and a plausible 3D view are useful checks, but neither proves that a board will work.

## Scope and design freedoms

The controller needs a practical arrangement for two joysticks, battery/switch access, Teensy USB/programming access, and the radio module. The OLED, battery sensing, and status LED remain optional. You may choose different regulators, resistor/capacitor/inductor package sizes, or through-hole parts if their ratings, pad mappings, and assembly fit are checked. Include any custom libraries you import. Two copper layers are recommended for cost; JLCPCB's published capabilities should govern the actual clearances, holes, and fabrication choices.

The supplied integrated joystick footprint combines each module's five connector pads and four module mounting holes. Do not add eight separate joystick-hole symbols/footprints on top of it. Check the real joystick module's connector order and board orientation before routing. The outer four holder holes are a separate mechanical feature.

The supplied holder STL is optional. To fit it, use the recommended 132.0 × 67.5 mm board outline, 1.6 mm board thickness, 3.0 mm-radius rounded corners, and four 3.8 mm non-plated outer holes. Measured from the outline's upper-left bounding corner (X right, Y down), hole centers are `(4.5, 4.0)`, `(127.5, 4.0)`, `(4.5, 63.5)`, and `(127.5, 63.5)` mm. These are holder-fit recommendations, not requirements if you design a different enclosure.

## Questions your review should answer

- What component grouping, orientation, and placement order would make routing easier while keeping the joysticks usable, switch and battery reachable, Teensy USB accessible, and any OLED viewable? Check the tentative positions in the 3D view.
- Is the buck's switching loop compact and placed per the regulator's layout guidance? Are radio decoupling and antenna clearance appropriate for the exact module?
- Are power and return paths intentional? A continuous bottom GND plane is often useful; a top supply plane such as 5 V may help where it makes sense. Check each zone's assigned net, islands, clearances, and antenna keepout.
- Are track widths selected by each net's current, copper weight, temperature rise, and signal needs rather than left at one default? Do clearances and vias meet JLCPCB's chosen process?
- Do every pad, track, and zone carry the intended net after updating from the schematic? Are there unrouted connections, DRC errors, mechanical collisions, or missing 3D model paths?

## Prompt to use with your AI

> Open my attached KiCad project and Part 2 guide. Inspect the actual PCB, schematic, footprint libraries, 3D models, and DRC report. If I have not placed the parts, propose a placement order and candidate groups or orientations that make routing practical. If I have placed them, critique their positions. Review component access, holder fit if I chose the supplied STL, integrated joystick footprints, regulator loop and returns, radio antenna area, planes and zone nets, per-net trace widths, clearances, and unrouted items. Rank possible problems by severity. For each, cite the board feature/net and the manufacturer or component rule I should check. Tell me what cannot be verified without measurements, the exact module, or fabrication settings; do not declare the board manufacturing-ready.
