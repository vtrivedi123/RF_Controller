# Lab 7 — Part 3: final project-ZIP AI context

Part 3 is a portability and final-review check, not a request for a written report. Submit one ZIP containing your completed KiCad project. You do **not** submit a DOCX, separate schematic PDF, or Gerbers for Lab 7. Board ordering waits until after Lab 8, when the RF controller and flight-controller boards can be ordered together.

The archive should contain the completed `.kicad_pro`, `.kicad_sch`, and `.kicad_pcb`, both project library tables, and every project-specific symbol, footprint, and 3D model used. Keep relevant supplied datasheets and the optional AI guides/firmware reference with the project if you used them. Do not include lock files, unrelated downloads, or an instructor reference board. A project that opens only on your computer is not portable.

## Final checks

1. Save the schematic and PCB, refill copper zones, and close KiCad before making the ZIP.
2. Run ERC and DRC on **your** project. Resolve errors; examine any warnings and confirm they are intentional. A clean ERC/DRC does not prove all design decisions are correct.
3. Extract the ZIP to a different folder and open that copied `.kicad_pro`. Check that the schematic, PCB, custom footprints, symbols, and 3D models load through project-relative paths.
4. Inspect the netlist and board once more for wrong voltage domains, omitted radio signals, connector pin-order mistakes, footprint mismatches, and unrouted pads. Confirm optional blocks omitted from hardware are also accounted for in your firmware plan.
5. Keep the source project and a copy of the tested ZIP. Do not order or attach motors/propellers as part of this check.

## Prompt to use with your AI

> I uploaded my final Lab 7 KiCad project ZIP. Extract it to a new folder and open the copied project files. Check that the archive is portable and complete, then review the schematic, PCB, libraries, ERC/DRC reports, net assignments, footprint pad mapping, 3D fit, and any design assumptions I state. Give a short prioritized list of confirmed errors, likely risks, and facts you cannot verify. Do not treat a successful file open or clean ERC/DRC as proof the circuit is safe to manufacture. Do not rewrite the design without asking me.
