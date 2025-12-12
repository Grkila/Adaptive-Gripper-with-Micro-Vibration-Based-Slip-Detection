# Adaptive Gripping System for Robotic Applications

![License](https://img.shields.io/badge/license-MIT-blue.svg) ![Platform](https://img.shields.io/badge/platform-ESP32%20%7C%20Python-orange)

## 📖 Overview

This repository contains the implementation of an **Adaptive Gripping System**, developed as part of a Master's Thesis. The system utilizes advanced signal processing (FFT) and slip detection algorithms to adjust gripping force dynamically in real-time.

The project combines embedded C++ control logic (ESP32) with a Python-based GUI for signal analysis and data visualization.

**Thesis:** [Download Full Thesis (PDF)](docs/Thesis_DIPLOMSKI.pdf)

## 📂 Project Structure

This repository is organized as follows:

```text
Adaptive_Gripping_Thesis/
├── firmware/                   # ESP32 C++ source code
│   └── ADAPTIVE_GRIPPING_FINAL/ # Arduino Sketch Folder
│       ├── src/                 # Logic, Drivers, and Config
│       └── ADAPTIVE_GRIPPING_FINAL.ino
├── software/                   # Python tools and analysis scripts
│   ├── signal_analysis_gui.py  # Real-time signal analysis dashboard
│   └── requirements.txt        # Python dependencies
├── hardware/                   # Mechanical design & Electronics
│   ├── pcb/                    # EasyEDA project files & Gerbers
│   ├── gripper_assembly.step   # 3D CAD models
│   └── (other CAD files)
├── docs/                       # Documentation
│   └── Thesis_DIPLOMSKI.pdf    # Master's Thesis document
├── data/                       # Recorded sensor logs
└── README.md
```

## 🚀 Key Features

*   **Slip Detection:** Real-time monitoring of object stability using frequency analysis (FFT) on magnetic sensor data.
*   **Adaptive Force Control:** PID-based regulation of gripping force based on sensor feedback.
*   **High-Speed Control Loop:** Deterministic 2kHz control loop using ESP32 hardware timers.
*   **PC Interface:** PyQt6-based dashboard for visualizing sensor streams and adjusting parameters.

## 🛠️ Hardware Setup

*   **Microcontroller:** ESP32 (utilizing dual-core capabilities)
*   **Sensors:** 
    *   Magnetic Hall Effect Sensors (for slip detection)
    *   Current Sensors (for force feedback)
*   **Actuators:** 
    *   Servo Motors (gripper actuation)
    *   Stepper Motors (Z-axis movement)
*   **PCB:** Custom PCB designed in EasyEDA (files in `hardware/pcb`).
*   **Input:** Physical buttons for manual control.

## 💻 Installation & Usage

### 1. Firmware (ESP32)
1.  Navigate to the `firmware` folder.
2.  Open `ADAPTIVE_GRIPPING_FINAL.ino` in the Arduino IDE or PlatformIO.
3.  Ensure the correct board (ESP32 Dev Module) is selected.
4.  Connect your board and upload the code.

### 2. Software (Python Analysis)
Ensure you have Python installed, then set up the environment:

1.  Navigate to the `software` folder:
    ```bash
    cd software
    ```
2.  Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```
3.  Run the GUI application:
    ```bash
    python signal_analysis_gui.py
    ```

## 📊 Methodology

The core logic resides in `firmware/src/Logic`. The system operates on a Finite State Machine (FSM) defined in `GrippingFSM.cpp`, transitioning between:
*   **IDLE:** Waiting for user input.
*   **APPROACH:** Positioning the gripper.
*   **GRIPPING:** Applying initial contact force.
*   **HOLDING:** Adaptive phase; monitors slip via FFT and increases force if instability is detected.
*   **RELEASE:** Releasing the object.

## 📄 License

This project is open-source. Please see the [LICENSE](LICENSE) file for details.

---
*Author: Dusan*
*Date: December 2025*
