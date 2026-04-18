# COMB — Common Open Modular robotic platform for Bees

> An open-source, modular robotic platform for experimental access to honey bee colonies.

COMB provides firmware, tools, and design files for building robots that can operate inside living honey bee hives. The platform was developed as part of the EU H2020-funded **Hiveopolis** project and supports multiple robot types — including a waggle-dance robot that mimics bee dances to communicate foraging locations, a scanner robot for inspecting comb structures, and a wing-flapper robot that generates local vibratory signals with flexible PCB wings. By keeping every subsystem modular and every component open-source, COMB lets researchers swap, extend, and reproduce experimental setups without starting from scratch.

Hardware files (CAD, PCB designs, and mechanical parts) are available on the [Zenodo Repository](https://zenodo.org/records/13693195).

---

## Demo
<video src="https://github.com/user-attachments/assets/8cfa3cd7-d7c5-49a2-9e20-cec200af73bd" controls width="600"></video>
<!-- <p align="center">
  <a href="https://www.youtube.com/watch?v=ClRFNMyWJLM">
    <img src="https://img.youtube.com/vi/ClRFNMyWJLM/maxresdefault.jpg" alt="COMB Platform Demo" width="600"/>
  </a>
  <br/>
  <em>▶ Click the image above to watch the demo on YouTube</em>
</p> -->
---

## Motivation

Honey bees are essential pollinators, and understanding their in-hive behaviour is critical for colony health monitoring and ecological research. Traditional observation methods are invasive and hard to scale. COMB addresses this by providing a standardised robotic toolkit that can interact with bees inside the hive — performing dances to guide foragers, scanning comb surfaces, or generating wing-buzz signals — all under experimental control.

---

## Key Features

- **Multi-robot firmware** — ready-to-flash controller code for three distinct robot types (dance, scanner, wing-flapper), each targeting a different experimental modality.
- **Modular architecture** — common hardware interfaces and pin-mapping conventions let you mix and match subsystems (motors, sensors, actuators, UI) without rewiring.
- **Virtual motor encoder** — a Teensy-based utility that emulates quadrature encoder signals, useful for bench-testing controllers without physical motors.
- **Keypad module** — standalone firmware for a modular user-input keypad, enabling in-field parameter adjustments.
- **MATLAB dance model** — scripts to define waggle-dance trajectory models and export them directly to C++ header variables for embedding in firmware.
- **Fully open-source** — all software is released under the MIT License; hardware files are publicly available on Zenodo.

---

## Repository Layout

```
COMB/
├── Controller_Code/
│   ├── DanceRobot/                    # Waggle-dance robot firmware (ESP-IDF)
│   ├── ScannerRobot/                  # Comb scanner robot firmware (ESP-IDF)
│   ├── WingFlapperSanDiego/           # Wing-flapper robot firmware (ESP-IDF)
│   └── README.md                      # Controller-specific documentation
│
├── Encoder_Code/
│   └── virtual_motor_encoder_teensy/  # Teensy sketch for emulating encoder signals
│
├── Keypad_Design_Code/
│   ├── Keypad/                        # Keypad firmware and build files
│   └── Readme.md
│
├── Dance_Model_Matlab/
│   ├── dance_model.m                  # MATLAB model of the waggle-dance trajectory
│   └── write_dance_model_to_cpp_variable.m  # Export model to a C++ variable
│
├── LICENSE
└── README.md
```

---

## Requirements

| Component | Toolchain / Software |
|---|---|
| Controller firmware (DanceRobot, ScannerRobot, WingFlapper) | [ESP-IDF](https://docs.espressif.com/projects/esp-idf) on an ESP32-compatible board |
| Virtual encoder | [Arduino IDE](https://www.arduino.cc/en/software) or PlatformIO with a Teensy board |
| Keypad firmware | Arduino IDE or PlatformIO with an Arduino-compatible board |
| Dance model scripts | [MATLAB](https://www.mathworks.com/products/matlab.html) R2020a or later |

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/praked/COMB.git
cd COMB
```

### 2. Build and flash controller firmware

Each robot under `Controller_Code/` is an ESP-IDF project. To build and flash, for example, the DanceRobot:

```bash
cd Controller_Code/DanceRobot
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash
```

Replace `<PORT>` with your serial port (e.g. `/dev/ttyUSB0` on Linux, `COM3` on Windows). Repeat the same process for `ScannerRobot` or `WingFlapperSanDiego`.

### 3. Virtual encoder (Teensy)

Open `Encoder_Code/virtual_motor_encoder_teensy/virtual_motor_encoder_teensy.ino` in the Arduino IDE, select your Teensy board under **Tools → Board**, and upload. This sketch emulates quadrature encoder pulses so you can test motor controllers on the bench.

### 4. Keypad firmware

Open the project under `Keypad_Design_Code/Keypad/` in the Arduino IDE. Adjust pin mappings to match your wiring, then compile and upload.

### 5. MATLAB dance model

```matlab
cd('Dance_Model_Matlab');
addpath(genpath(pwd));
dance_model;                          % Run the dance trajectory model
write_dance_model_to_cpp_variable;    % Export the model as a C++ variable
```

The exported C++ variable can then be included directly in the controller firmware to define the robot's waggle-dance path.

---

## Configuration Notes

- **Pin mappings and constants** — adjust GPIO assignments in the header files (`commands.h`, `gpio.h`, and the main firmware source) to match your specific hardware wiring.
- **Encoder resolution** — the virtual encoder's ticks-per-revolution setting must match what the controller firmware expects. Verify both sides before running closed-loop tests.
- **Power and grounding** — ensure all modules share a common ground and that voltage rails are correct for each board (3.3 V for ESP32, 5 V or 3.3 V for Teensy depending on variant).

---

## Project Context

COMB was developed at the **Biorobotics lab at Free University of Berlin, Germany** as part of the [Hiveopolis](https://hiveopolis.eu/) project, an EU Horizon 2020 initiative exploring how robotic systems can interact with and support honey bee colonies. The waggle-dance robot, for instance, is designed to mimic the stereotypical figure-eight dance that forager bees perform on the comb surface to communicate the direction and distance of food sources. By reproducing this dance with a robot inside a live hive, researchers can study how bees decode these signals and potentially guide colonies toward safe foraging sites.

---

## Contributing

Contributions are welcome. If you'd like to add a new robot module, improve existing firmware, or contribute documentation, please open an issue or submit a pull request.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Acknowledgements

This work was supported by the European Union's Horizon 2020 research and innovation programme. Hardware design files are archived on [Zenodo](https://zenodo.org/records/13693195).
