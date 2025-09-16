# COMB — Common Open Modular robotic platform for Bees

Open-source, modular hardware + firmware to non-invasively access and study honey-bee colonies using low-cost, reproducible components.

> License: MIT

---

## Why COMB

- Modular: swap subsystems without rewiring the whole rig.
- Open: C/C++ firmware and MATLAB tools included.
- Reproducible: documented build and flashing steps.
- Extensible: add sensors, actuators, and UI modules as needed.

---

## Repository layout

COMB/
├─ Controller_Code/ # Main microcontroller firmware
├─ Encoder_Code/
│ └─ virtual_motor_encoder_teensy/ # Teensy-based virtual encoder utility
├─ Keypad_Design_Code/ # Keypad UI firmware
├─ Dance_Model_Matlab/ # MATLAB models and analysis
├─ LICENSE
└─ README.md

markdown
Copy code

---

## Hardware overview

- **MCU:** Teensy family (for virtual encoder) and common Arduino-compatible boards for controller/keypad.
- **Actuation & IO:** Motor/servo drive, quadrature encoder (virtual encoder available for bench testing), keypad input.
- **Sensing (typical):** Temperature, weight/force, positional encoders.  
- **Power:** Regulated 5 V and motor rail as per your actuator specs.

> Substitute equivalent boards and sensors if pin mapping is updated in code.

---

## Software overview

- **Languages:** C (primary), C++, MATLAB.
- **Build systems:** Arduino IDE or PlatformIO; MATLAB for analysis scripts.
- **Targets:**  
  - `Controller_Code/` → main colony-interface controller.  
  - `Encoder_Code/virtual_motor_encoder_teensy/` → signal source for testing without a physical encoder.  
  - `Keypad_Design_Code/` → human-input module.  
  - `Dance_Model_Matlab/` → models and utilities for data exploration and validation.

---

## Quick start

### 1) Tooling

- **Arduino IDE** or **PlatformIO** with:
  - Teensy core (if using Teensy)
  - Board support for your controller MCU
- **MATLAB** (R2020a+) for `Dance_Model_Matlab/`

### 2) Clone


git clone https://github.com/praked/COMB
cd COMB
3) Build + Flash (examples)
Controller
Open Controller_Code in Arduino IDE or PlatformIO.

Select the correct board and port.

Verify and upload.

Virtual Encoder (Teensy)
Open Encoder_Code/virtual_motor_encoder_teensy.

Select a Teensy board (e.g., 3.x/4.x).

Upload and use as a quadrature source to exercise your control stack.

Keypad
Open Keypad_Design_Code.

Set pin map to match your wiring.

Upload.

4) MATLAB tools
matlab
Copy code
cd('Dance_Model_Matlab');
% Add to path if needed
addpath(genpath(pwd));
% Run demo or scripts as documented in that folder
Configuration
Pins and constants: Defined in each sketch/source. Adjust to your wiring before flashing.

Baud rates: Keep serial settings consistent across modules.

Encoder resolution: Match virtual encoder ticks per rev with your controller expectations.

Typical wiring (text summary)
MCU digital pins → encoder A/B (or virtual encoder outputs)

MCU PWM/driver pins → motor/servo driver inputs

Keypad matrix → GPIO rows/cols defined in Keypad_Design_Code

Sensors → analog/digital inputs per module

Common ground across all modules and power rails

Development workflow
Bring-up with virtual encoder to validate control loops.

Integrate keypad to exercise UI states and parameter entry.

Swap in real encoder and live sensors.

Use MATLAB models to validate expected behaviors and compare logs.

Testing
Unit-level: Use virtual encoder to step through speeds and directions, confirm counts and debouncing.

Integration: Confirm keypad inputs change controller states safely; verify actuator saturation limits.

Safety: Include current limits, soft-stops, and watchdogs. Never test motors on a live colony environment without fail-safes.
