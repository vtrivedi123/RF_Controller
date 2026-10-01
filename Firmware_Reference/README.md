# Transmitter firmware reference for Lab 7

`Teensy_RF_Controller_TX_Student_Reference/` contains a pin-redacted copy of the instructor's Teensy 4.0 transmitter firmware and its local `NonBlockingUsb.h` dependency. It is supplied now so you can see which hardware interfaces the eventual firmware uses. **It is not a Lab 7 deliverable and is intentionally not ready to compile or flash.** The original instructor pin assignments and battery-sense divider values are not supplied here.

The code uses four joystick ADC inputs, two joystick-switch inputs, an nRF24L01+ over SPI with CE and CSN control, and optional OLED, battery-sense, and status-LED functions. The optional OLED, battery sensing, status LED, and debug PWM outputs are disabled by default in this student copy; enable only the features your board actually has. Debug PWM outputs are not required on your Lab 7 PCB. The radio protocol and live-PID-tuning features are present for context, but firmware operation is addressed later in the course.

Before adapting or compiling it later:

1. Choose Teensy pins from the official pinout for **your** schematic, including a compatible SPI interface. Match the pin map to the schematic and PCB; do not choose pins solely because a name appears in this file.
2. Fill every `UNASSIGNED_PIN` needed by your design. Enable optional feature flags only for blocks you include. Set the battery-divider constants from your own selected resistors if you enable battery sensing.
3. Verify joystick connector order and physical axis direction on the assembled board. The genericized axis mapping and calibration directions in this copy are starting points, not a tested mapping for your layout.
4. Remove the deliberate `#error` only after reviewing the assignments. Build for Teensy 4.0 with the RF24, Adafruit GFX, and Adafruit SSD1306 libraries; SPI, Wire, and Servo come from the Teensy/Arduino toolchain.
5. Keep motors and propellers disconnected during controller bring-up. A successful compile does not establish electrical safety or correct control direction.

This reference does not change the Lab 7 submission: submit the completed, portable KiCad project ZIP. You are not asked to demonstrate working firmware in Lab 7.
