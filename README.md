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

```text
COMB-main/
├─ Controller_Code/                 # Firmware for different robot modules
│  ├─ DanceRobot/                   # Dance communication robot
│  ├─ ScannerRobot/                 # Scanner robot for colony mapping
│  ├─ WingFlapperSanDiego/          # Wing flapper prototype
│  └─ README.md
│
├─ Encoder_Code/
│  └─ virtual_motor_encoder_teensy/ # Teensy sketch emulating motor encoder signals
│
├─ Keypad_Design_Code/
│  ├─ Keypad/                       # Keypad firmware and build files
│  └─ Readme.md
│
├─ Dance_Model_Matlab/              # MATLAB models and tools
│  ├─ dance_model.m
│  └─ write_dance_model_to_cpp_variable.m
│
├─ LICENSE
└─ README.md
```

---

## Features

- **Controller firmware** for multiple robot types (dance, scanner, wing-flapper).
- **Virtual encoder utility** to simulate quadrature signals using a Teensy board.
- **Keypad firmware** for modular user input.
- **MATLAB scripts** to define and export dance models to C++.
- Fully open-source under MIT License.

---

## Requirements

- **MCUs:** ESP-IDF supported boards (for controller code), Teensy (for encoder utility), Arduino-compatible boards (for keypad).
- **Toolchains:**
  - [ESP-IDF](https://docs.espressif.com/projects/esp-idf) for controller code.
  - [Arduino IDE](https://www.arduino.cc/en/software) or PlatformIO for Teensy and keypad.
  - [MATLAB](https://www.mathworks.com/products/matlab.html) (R2020a+) for model scripts.

---

## Quick start

### 1. Clone

```bash
git clone https://github.com/praked/COMB
cd COMB

```
### 2. Build firmware
- DanceRobot / ScannerRobot / WingFlapperSanDiego
  Use ESP-IDF:
```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash
```

- Virtual encoder (Teensy)
  Open Encoder_Code/virtual_motor_encoder_teensy.ino in Arduino IDE, select your Teensy board, and upload.

- Keypad firmware
  Open Keypad_Design_Code/Keypad in Arduino IDE, set pin mapping, and upload.

### 3. MATLAB tools
```matlab
cd('Dance_Model_Matlab');
addpath(genpath(pwd));
dance_model
write_dance_model_to_cpp_variable
```
