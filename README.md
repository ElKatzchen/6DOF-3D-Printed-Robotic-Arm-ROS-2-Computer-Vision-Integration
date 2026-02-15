# 6DOF 3D-Printed Robotic Arm: ROS 2 & Computer Vision Integration

This repository contains all the necessary data to fabricate, control, and modify a **6-Degree-of-Freedom (6DOF) robotic arm**. The project combines 3D printing, advanced CAD modeling, and a distributed control system using ROS 2.

## 🚀 Overview

The system is designed for **real-time teleoperation** via computer vision. It tracks a user's hand movements in a dedicated vision area and translates those coordinates into precise robotic motion using inverse kinematics.

### Key Features:
* **Mechanical Design:** All components are 3D-printable and were fully modeled in **SolidWorks 2025**. Original design files are included for easy modification.
* **Communication Stack:** Powered by **ROS 2 Humble**, enabling seamless distributed processing between a **PC** (high-level processing) and a **Raspberry Pi** (hardware abstraction).
* **Computer Vision:** Utilizes a dual-camera setup to track hand **keypoints** (Mediapipe-based).
* **Inverse Kinematics (IK):** The system captures the $XYZ$ coordinates of the user's index finger and maps them from the "Vision Space" to the "Robot Workspace," calculating the required joint angles in real-time.

---

## 📋 Prerequisites

Before you begin, ensure you have the following software and tools installed:

### **CAD & Hardware Design**
* **SolidWorks 2025**: Required to open or modify the original `.SLDPRT` and `.SLDASM` files.
* **3D Printer Slicer**: (e.g., Ultimaker Cura or PrusaSlicer) for processing the STL files.

### **Software & Robotics OS**
* **Ubuntu 22.04 LTS**: The recommended OS for stable ROS 2 Humble integration.
* **ROS 2 Humble Hawksbill**: The primary communication middleware.
* **Python 3.10+**: For vision and kinematics scripts.

### **Computer Vision & Libraries**
To run the hand-tracking and coordinate transformation nodes, you will need:
* **OpenCV**: For image processing.
* **MediaPipe**: For hand keypoint detection and tracking.
* **NumPy**: For matrix operations and inverse kinematics calculations.

---

## 📂 Repository Structure

The repository is organized into several folders:
* `/3D-Printed_Files`: Includes STL files for printing and SolidWorks 2025 source files.
* `/REDACTED`: The ROS 2 workspace containing the packages for robot description and control.
* `/REDACTED`: Python scripts for dual-camera hand tracking and coordinate mapping.
* `/REDACTED`: Wiring diagrams and Raspberry Pi configuration.

---

## 🛠️ Work in Progress

This project is currently under active development. Future updates will include:
1. Full assembly guide and Bill of Materials (BOM).
2. Calibration scripts for the dual-camera setup.

---

## 📌 Current Status: v0.2.2 (Alpha)
* **Current Version:** 0.2.2-beta
* **Last Update:** February 2024
* **Stable Version:** None yet (Under Development)

---

**Author:** [Katzchen]  
