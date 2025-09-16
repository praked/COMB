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
COMB/
├─ Controller_Code/                 # Main microcontroller firmware
├─ Encoder_Code/
│  └─ virtual_motor_encoder_teensy/ # Teensy-based virtual encoder utility
├─ Keypad_Design_Code/              # Keypad UI firmware
├─ Dance_Model_Matlab/              # MATLAB models and analysis
├─ LICENSE
└─ README.md
