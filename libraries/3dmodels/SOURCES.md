# 3D model sources

The PCB uses local copies so the KiCad project remains portable.

- `EG1218_official_E-Switch.step` — official E-Switch EG1218 STEP model: https://configured-product-images.s3.amazonaws.com/stp3dmodels/EG1218.stp
- `L_Bourns_SRP5030T_KiCad.step` — KiCad library model for the dimensionally compatible Bourns SRP5030T family. The populated part is SRP5030HMCT-150M; Bourns product/design-file references: https://www.bourns.com/products/magnetic-products/product-detail/power-inductors-smd-high-current-shielded/srp5030hmct
- `SSD1306_0.96in_with_headers_community.step` — premade 0.96-inch SSD1306 module with headers from the-this-pointer/kicad-oled-ssd1306-128x64: https://github.com/the-this-pointer/kicad-oled-ssd1306-128x64

The nRF24L01 and joystick models remain provisional until exact matching source models are obtained. Do not substitute a visually similar model without checking module dimensions, connector pitch/origin, and component orientation against the footprints.
