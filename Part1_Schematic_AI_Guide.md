# Lab 7 — Part 1: schematic-design AI context

Attach this file **with your own KiCad project** when asking an AI assistant to review your schematic. The handout and component datasheets in `Datasheets/` remain the assignment and primary technical sources; this guide is context, not an answer key. Ask the AI to find relevant datasheet sections, apply their equations and ratings to your requirements, compare candidate passive values, and cite the specific symbol pin, net, datasheet section, or ERC finding behind each concern. Make the final design decisions yourself.

## What the controller must accomplish

This is a handheld RF transmitter, not the aircraft flight controller. Two analog joysticks provide four axes and two press switches. A Teensy 4.0 reads those inputs, forms control packets, and communicates with an nRF24L01+ radio. It needs a battery-powered path and suitable regulated supplies. The instructor architecture starts from a 9 V alkaline battery, steps down to a 5 V controller rail, then uses a separate 3.3 V rail for the radio. Joystick outputs must remain within the Teensy's input-voltage limits; do not assume a pin labelled `+5v` on a joystick module means it should be powered at 5 V here.

The starter contains placed symbols but intentionally omits connections and resistor, capacitor, and inductor values. You select values from the actual component documentation and choose the Teensy pin map. The supplied component choices, especially the regulators, are suggestions rather than an immutable bill of materials. The battery-sense divider, RF-status LED, and OLED are optional; if you omit a block, remove its schematic parts and plan to disable its firmware feature. Do not invent unused circuitry merely to match the starter.

## Questions your review should answer

- Does every power rail have a defined source, return, voltage domain, and suitable decoupling? Are input/output ratings and expected current supported by the chosen parts?
- Do the switch, polarity-protection device, buck stage, and radio regulator match their exact datasheets, including pin functions and selected external components?
- Are four analog axes, two press switches, radio SPI/control, and any optional features mapped to Teensy pins that actually support those functions? Is every external signal safe for 3.3 V logic?
- Do symbol pin numbers, connector order, and assigned footprints agree? The integrated joystick footprint includes a five-pad connector and four mounting holes; the four standalone `H1`–`H4` holes are for the outer holder pattern.
- Which ERC errors remain? A net label must electrically touch its intended wire or pin. A visually nearby label is not a connection. Review warnings instead of hiding them.

The pin-redacted firmware under `Firmware_Reference/` describes interface needs, **not** the instructor's MCU pins and not a finished Lab 7 solution. It is deliberately non-flashable until adapted to your own board.

## Prompt to use with your AI

> Open my attached KiCad project, the Lab 7 handout, relevant datasheets, and `Firmware_Reference/README.md`. Review Part 1 as a design reviewer. Check the power architecture, component values and ratings, joystick and radio pin mapping, Teensy pin capabilities and 3.3 V limits, optional-block choices, footprint pin numbers, and ERC findings. List the most serious suspected issues first, with file/component/net evidence and the datasheet fact I should verify. Separate confirmed errors from uncertainties. Do not assume the instructor's pin map. Propose candidate passive values using my requirements and the supplied datasheets; cite the equations, tables, assumptions, and ratings behind each recommendation so I can verify and choose the final parts.
