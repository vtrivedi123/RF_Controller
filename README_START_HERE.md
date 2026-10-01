# Lab 7 RF controller student starter

Extract the entire ZIP before opening `RF Controller V-Final.kicad_pro` in KiCad. The handout is also included in `Documentation/`.

The schematic contains the supplied components with no electrical connections. Resistor, capacitor, and inductor values are intentionally `TBD`. The PCB file is empty. These are the starting conditions for the design assignment.

## Files to use

- `Documentation/Lab7_RF_Controller_PCB_Design_Student_Handout.docx`: the assignment, reference images, design guidance, and submission requirements.
- `Part1_Schematic_AI_Guide.md`: upload with your schematic and relevant datasheets for help selecting values, checking nets, and choosing MCU pins.
- `Part2_PCB_AI_Guide.md`: upload with your project for help planning component placement and reviewing routing and mechanical fit.
- `Part3_Submission_AI_Guide.md`: use when checking the final ZIP and project portability.
- `Datasheets/`: 11 PDF references, `START_HERE.md` explaining which document to use, and `SOURCES.json` recording their sources. These are the component documentation referred to in the handout. Verify exact module variants and obtain documentation for any substituted parts.
- `Firmware_Reference/`: pin-redacted transmitter code, its local header, and instructions. It is for interface planning and is intentionally not ready to compile or flash.
- `mechanical/RF_Controller_Holder.stl`: the optional holder reference.
- `libraries/`, `sym-lib-table`, and `fp-lib-table`: project libraries and 3D models. Keep them together. The standard symbols and footprints used by this starter are bundled alongside the custom libraries.

Markdown (`.md`) files are plain text: open them in a text editor, or attach them to your AI conversation. Give the AI the guide for your current part, the actual project files, and relevant datasheets. Ask for cited calculations, alternatives, placement suggestions, and specific concerns. Check the evidence before accepting its recommendations.

## Opening the project

The project was saved with KiCad 10.0. Use a compatible KiCad version; KiCad 10 is recommended. The ZIP contains project assets, not the KiCad application. Project-local paths resolve from `${KIPRJMOD}`. The bundled standard footprint libraries contain only the footprints used here; if you add different components, add their assets or use your installed KiCad libraries and make the final submission portable.

Some supplied module models are approximate visual references. Confirm physical dimensions and pad mappings against your actual parts. Bare mounting holes and solder pads do not need a separate component body.

## Submission

Follow the handout: submit your completed, portable KiCad project ZIP. No separate design-reasoning report, DOCX submission, schematic PDF, or Gerbers are required for Lab 7. Board ordering follows Lab 8. The firmware reference is not a Lab 7 programming task.
