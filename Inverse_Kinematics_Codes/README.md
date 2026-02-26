# 📐 Inverse Kinematics & Control System

This folder contains the core logic for calculating joint angles and the communication bridge between the high-level Python processing and the low-level hardware control.

## 🧠 Kinematics Engine (Python)

The main script handles the mathematical transformation from Cartesian coordinates $(X, Y, Z)$ to joint angles $(\theta_1 ... \theta_6)$.

* **IK Solver:** A Python-based script that calculates required angles in real-time.
* **Communication:** Data is transmitted via **UART** (Serial) to the microcontroller.
* **Software Safety Layer:** The script includes a pre-transmission validation check. It prevents the system from sending commands that exceed the mechanical limits of the arm or positions that are mathematically unreachable (Singularities).

---

## 🎮 Hardware Control (ESP32 / Arduino)

The low-level control is designed for an **ESP32 (30-pin version)**. This folder includes two distinct firmware options:

### 1. Home Calibration Script (`home_setup.ino`)
A standalone script used for initial calibration. When executed, it moves all servos to their "Home" or "Rest" positions. This is essential for verifying mechanical alignment before running full autonomous control.

### 2. UART Receiver & Servo Controller (`main_controller.ino`)
This script works in tandem with the Python IK solver. It listens for incoming angle data via UART and maps it to the servo PWM signals.
* **Redundant Safety System:** To ensure system reliability, the ESP32 performs a secondary validation on all incoming data. If a received angle is outside the physical range of the motors, the command is ignored or clamped, preventing hardware damage.

---

## 🛠️ Control Workflow

1.  **Vision Input:** Hand coordinates are captured.
2.  **IK Calculation:** Python computes the angles.
3.  **Safety Check 1:** Python validates the reachable workspace.
4.  **UART Transmission:** Angles are sent to the ESP32.
5.  **Safety Check 2:** ESP32 verifies signal integrity and physical limits.
6.  **Actuation:** Servos move to the validated positions.

---

## 📌 Pinout & Hardware Notes (ESP32)
* **UART RX/TX:** Ensure common ground between the PC/Raspberry Pi and the ESP32.
* **Power Supply:** Use an external power source for the servos (don't power 7 servos directly from the ESP32). The power supply used is 5VDC 10A.

**Next Step:** For details on how the coordinates are captured, see the [**Vision System**](../Vision_System) documentation.
