# Lab 7 component documentation

“Component documentation” means the manufacturer's datasheets, pinout references, and application guidance—not the lecture notes. This folder supplies offline PDF references for the main parts discussed in the handout. Source URLs are recorded in `SOURCES.json`.

## Where to start

| Circuit block or decision | Open this file | Look for |
|---|---|---|
| 5 V buck regulator and its surrounding resistor/capacitor/inductor choices | `TI_TPS542025_Buck_Regulator.pdf` | The TPS542025 variant, pin functions, application/design procedure, inductor and capacitor selection, enable behavior, ratings, and layout. The family also includes TPS542021: do not mix output variants. |
| Dedicated radio 3.3 V regulator | `Diodes_AP2112_3V3_Regulator.pdf` | AP2112K-3.3 package/pins, input/output capacitor requirements, current, dropout, stability, and thermal constraints. |
| Inductor candidate ratings and physical fit | `Bourns_SRP5030HMCT_Inductor_Family.pdf` | Family selection table, inductance, saturation/RMS current, resistance, dimensions and recommended pads. First determine the circuit's needs using the regulator documentation; then evaluate a suitable part. |
| MCU pins and voltage domains | `PJRC_Teensy40_Pinout_Front.pdf` and `PJRC_Teensy40_Pinout_Back.pdf` | ADC/SPI/I2C pin functions, supply pins, voltage limits, and power notes. Full board documentation: https://www.pjrc.com/store/teensy40.html |
| Radio operation and electrical interface | `Nordic_nRF24L01Plus_Product_Specification.pdf` | Operating conditions, pin functions, SPI/control interface, and layout/decoupling guidance. This describes the IC, not the connector arrangement or complete current demand of every breakout module. |
| Status LED and series resistor | `Kingbright_APT2012LZGCK_LED.pdf` | LED voltage/current characteristics, polarity, and package. Combine these with the MCU output limits when selecting the resistor. |
| Battery switch | `ESwitch_EG1218_Switch_Drawing.pdf` | Contact arrangement, ratings, terminal numbering and mechanical drawing. |
| SS14 protection diode | `Vishay_SS14_Schottky_Example_Variant.pdf` | An example manufacturer's SS14 variant: forward drop, reverse voltage, current/thermal ratings, polarity and package. The starter does not identify a unique diode supplier; verify the exact part you select. |
| Joystick | `JoyIT_KY023_Joystick_Example_Module_Manual.pdf` | Example KY-023-style module operation. Verify the supplied module's markings, pin order, geometry and output range independently. Do not copy another MCU's wiring or supply voltage blindly. |
| Optional OLED | `Solomon_SSD1306_IC_Datasheet.pdf` | Display-controller interface and electrical information. This is the chip datasheet, not an exact four-pin module schematic. Verify the actual module's supply, connector order, onboard regulation and pull-ups. |

## How to use these references to choose passives

1. Identify what each resistor, capacitor or inductor does in your circuit.
2. Read the relevant IC's application guidance to determine the required value or acceptable range and the operating conditions behind it.
3. For an LED resistor, divider, pull-up or filter, use the connected devices' limits and the circuit's purpose; a resistor catalogue alone does not determine the correct resistance.
4. Select an actual component whose datasheet confirms its tolerance, electrical ratings, package and other relevant properties. For capacitors, check effective capacitance under bias as well as nominal capacitance and voltage rating. For inductors, check current ratings and resistance as well as inductance.
5. Recheck the calculation and the footprint against your chosen part. Example application values are not automatically correct for every design or substitution.

The generic resistors and capacitors intentionally have no assigned purchasable part numbers in the student design. Their final datasheets depend on the parts you select. These documents guide that choice; this folder is not a completed BOM or answer key.

## Module and substitution limits

The joystick manual is for a named example module, not a guarantee that every KY-023 clone has the same geometry or labeling. The radio and OLED documents describe their controller ICs, not all breakout boards. Use the exact module supplier's documentation when available, and resolve differences before connecting or ordering parts. If you substitute a regulator or another component, obtain the replacement's datasheet instead of applying these specifications to it.

These are reference copies collected September 27, 2026. Manufacturer documents retain their own revision dates and notices. Check for updates when selecting actual parts. Nordic's document is the manufacturer's specification hosted by MySensors; the Solomon document is the manufacturer's datasheet hosted by Adafruit. They are not new specifications written by those hosts.
