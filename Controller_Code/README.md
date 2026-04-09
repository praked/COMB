# Controller Code

Firmware for the three COMB robot modules — **DanceRobot**, **ScannerRobot**, and **WingFlapperSanDiego** — plus the **Keypad** input module. All four are ESP-IDF projects targeting the **ESP32** and are built with the same toolchain.

---

## ESP-IDF Version

This code was developed and tested against **ESP-IDF v4.3**. Using a different major version (especially v5.x) will likely cause build errors because the firmware relies on legacy timer and peripheral APIs (`driver/periph_ctrl.h`, `soc/timer_group_struct.h`, `timer_isr_callback_add`, etc.) that were restructured in later releases.

**Recommended:** install ESP-IDF **v4.3.x** (latest patch).

---

## Setting Up the Development Environment

### Option A — VS Code + ESP-IDF Extension (recommended)

1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Open the Extensions panel (`Ctrl+Shift+X`) and install the **C/C++ Extension**.
3. Install the **ESP-IDF Extension** (`Ctrl+Shift+X`, search for "Espressif IDF").
4. On first launch the extension will prompt you to install an ESP-IDF version — select **v4.3.x**.
5. The extension handles PATH, Python virtual-env, and toolchain setup automatically.

A video walkthrough (slightly outdated but still useful): [https://www.youtube.com/watch?v=Lc6ausiKvQM](https://www.youtube.com/watch?v=Lc6ausiKvQM)

### Option B — Command-line installation

Follow the official Espressif guide for your OS:

```bash
# Clone ESP-IDF v4.3
mkdir -p ~/esp
cd ~/esp
git clone -b v4.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh        # Linux / macOS
# or install.bat    # Windows

# Activate the environment (run this in every new terminal session)
. ./export.sh       # Linux / macOS
# or export.bat     # Windows
```

After activation, `idf.py --version` should report a 4.3.x version.

---

## Repository Structure

```
Controller_Code/
├── DanceRobot/
│   ├── CMakeLists.txt
│   └── main/
│       ├── main.c                          # Application entry point (app_main)
│       ├── dancegen.c / dancegen.h          # Waggle-dance trajectory generator
│       ├── dance_model_static_trajectory.h  # Pre-computed dance path (from MATLAB)
│       ├── gpio.h                          # Pin definitions (motors, limit switches, wings)
│       ├── motor.h                         # Motor constants (steps/rev, speed conversions)
│       ├── commands.h                      # UART protocol command IDs and system-state flags
│       └── types.h                         # Shared data structures (V3Df, WingsParam, etc.)
│
├── ScannerRobot/
│   ├── CMakeLists.txt
│   └── main/                              # Same file structure as DanceRobot
│       └── ...                            # Uses Z-axis instead of Y-axis; UART2 connects
│                                          #   to a Raspberry Pi instead of the encoder
│
├── WingFlapperSanDiego/
│   ├── CMakeLists.txt
│   └── main/
│       ├── main.c                         # Minimal firmware — toggles wing pins at ~50 Hz
│       ├── gpio.h                         # Only 3 pins: motor enable, wing IN1, wing IN2
│       ├── motor.h / commands.h / types.h
│       └── CMakeLists.txt
│
└── README.md                              # (this file)
```

---

## Robot Modules at a Glance

### DanceRobot

The main waggle-dance robot. Drives three stepper motors (rotation φ, linear X, linear Y) via step/direction GPIO pins, reads four homing limit switches, and actuates a flexible wing PCB. Communicates over three UART channels simultaneously:

| UART | Port | Baud | Pins (TX/RX) | Connected to |
|------|------|------|-------------|--------------|
| UART0 | 0 | 115200 | GPIO 1 / 3 | Host PC or GUI (serial monitor) |
| UART1 | 1 | 57600 | GPIO 17 / 16 | Keypad module |
| UART2 | 2 | 115200 | GPIO 26 / 25 | Virtual motor encoder (Teensy) |

### ScannerRobot

Structurally similar to the DanceRobot but uses a Z-axis motor instead of Y-axis (for vertical scanning). UART2 connects to a **Raspberry Pi** instead of the encoder, enabling higher-level scan coordination.

| UART | Port | Baud | Pins (TX/RX) | Connected to |
|------|------|------|-------------|--------------|
| UART0 | 0 | 115200 | GPIO 1 / 3 | Host PC or GUI |
| UART2 | 2 | 115200 | GPIO 26 / 25 | Raspberry Pi |

### WingFlapperSanDiego

A minimal standalone wing-flapper. No motion axes — it simply toggles two wing-drive pins (GPIO 25, GPIO 33) at a configurable frequency (default ~50 Hz). Useful for generating isolated wing-buzz stimuli.

---

## Building and Flashing

### 1. Navigate to the project

```bash
cd Controller_Code/DanceRobot    # or ScannerRobot, WingFlapperSanDiego,
                                 # or ../Keypad_Design_Code/Keypad
```

### 2. Set the target chip

```bash
idf.py set-target esp32
```

### 3. (Optional) Adjust configuration

```bash
idf.py menuconfig
```

You can tweak FreeRTOS tick rate, UART buffer sizes, log levels, and other SDK options here. For most cases the defaults are fine.

### 4. Build

```bash
idf.py build
```

### 5. Connect the ESP32 board

Plug the ESP32 board into your computer via USB. Identify the serial port:

| OS | Typical port |
|----|-------------|
| Linux | `/dev/ttyUSB0` or `/dev/ttyACM0` |
| macOS | `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART` |
| Windows | `COM3`, `COM4`, etc. (check Device Manager) |

If the port is not detected, you may need to install the USB-to-UART bridge driver for your board's chip (commonly CP2102 or CH340).

### 6. Flash

```bash
idf.py -p /dev/ttyUSB0 flash
```

Replace `/dev/ttyUSB0` with your actual port. Some boards require you to hold the **BOOT** button while pressing **EN/RST** to enter download mode.

### 7. Monitor serial output

```bash
idf.py -p /dev/ttyUSB0 monitor
```

Press `Ctrl+]` to exit the monitor. You can also combine flash and monitor in one step:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Wiring Overview

### DanceRobot / ScannerRobot — Motor Connections

All stepper motors are driven with external stepper drivers (e.g. A4988, TMC2208) that accept STEP and DIR signals. The ESP32 also controls a shared ENABLE pin.

| Function | GPIO | Notes |
|----------|------|-------|
| Motor Enable | 4 | Active-high; shared across all axes |
| φ (rotation) STEP | 19 | |
| φ (rotation) DIR | 21 | |
| X (linear) STEP | 22 | |
| X (linear) DIR | 23 | |
| Y STEP (DanceRobot) / Z STEP (Scanner) | 5 | |
| Y DIR (DanceRobot) / Z DIR (Scanner) | 18 | |

### DanceRobot / ScannerRobot — Limit Switches

| Switch | GPIO | Notes |
|--------|------|-------|
| X min | 34 | Input-only pin |
| X max | 15 | |
| Y/Z min | 39 | Input-only pin (VN) |
| Y/Z max | 13 | |

### DanceRobot / ScannerRobot — Wing Actuator

| Function | GPIO |
|----------|------|
| Wing IN1 | 32 |
| Wing IN2 | 33 |

### WingFlapperSanDiego

| Function | GPIO |
|----------|------|
| Motor Enable | 26 |
| Wing IN1 | 25 |
| Wing IN2 | 33 |

### Keypad Module

The keypad connects to the DanceRobot via **UART1** (57600 baud, GPIO 17 TX / GPIO 16 RX). It reads 10 physical buttons and sends a 2-byte status packet encoding direction, dance on/off, mode toggle, and speed up/down.

| Button | GPIO |
|--------|------|
| Z Counter-Clockwise | 15 |
| Z Clockwise | 19 |
| Down | 27 |
| Right | 18 |
| Up | 23 |
| Left | 2 |
| Dance On/Off | 4 |
| Mode | 5 |
| Speed Up | 34 |
| Speed Down | 35 |

The keypad also drives a serial LCD display (via I²C using the DFRobot_LCD library included in the source).

---

## Customisation

- **Pin mappings** — edit `gpio.h` in the relevant project's `main/` directory if your hardware wiring differs from the defaults above.
- **Motor parameters** — edit `motor.h` to change steps per revolution (default 200 full steps × 8 microsteps = 1600), axis travel per revolution (4.88 mm), or speed conversion factors.
- **Dance parameters** — edit default values in `dancegen.c` or regenerate `dance_model_static_trajectory.h` from the MATLAB scripts in `Dance_Model_Matlab/`.
- **Wing frequency** — in WingFlapperSanDiego, the timer alarm value (default 20000 µs → 50 Hz) is set in `app_main()`. Change `Frequency_divider` to adjust.
- **UART baud rates** — defined as `#define` constants at the top of each `main.c`. Make sure the connected device (PC, Teensy, Raspberry Pi, Keypad) uses the matching baud rate.

---

## Troubleshooting

- **Build fails with missing headers** (`driver/periph_ctrl.h`, `soc/timer_group_struct.h`) — you are likely using ESP-IDF v5.x. Switch to v4.3.
- **Port not found** — install the correct USB-to-UART driver (CP210x or CH340) for your ESP32 dev board.
- **Flash fails / times out** — hold the **BOOT** button while pressing **EN** to force the chip into download mode, then retry `idf.py flash`.
- **Motors don't move** — verify the ENABLE pin is driven high and that your stepper driver is wired correctly (STEP, DIR, GND).
- **No keypad data** — confirm UART1 baud rate is 57600 on both the keypad and the DanceRobot, and that TX/RX lines are crossed (TX→RX, RX→TX).

---

## License

MIT — see the repository root [LICENSE](../LICENSE) file.
