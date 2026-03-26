# 🤖 ROS 2 Workspace: Distributed Control System

This folder contains the **ROS 2 Humble** packages and nodes that manage communication, image streaming, and coordinate processing between the **Raspberry Pi** and the **Workstation (PC)**.

## 🏗️ System Architecture

The project follows a distributed architecture with **4 main nodes** designed to minimize latency and optimize processing power:

### 📡 Raspberry Pi Nodes (Edge)
* **Camera Publishers (2 Nodes):** These nodes capture real-time video streams from the dual-camera setup and publish them as compressed image topics to the network.
* **Servo Subscriber (1 Node):** This node listens for the calculated joint angles sent from the PC. Once received, it forwards the data via **UART** to the ESP32 microcontroller for physical actuation.

### 🖥️ Workstation Node (Central Processing)
* **Vision & IK Processor (1 Publisher/Subscriber Node):** This is the "brain" of the ROS 2 network.
    * **Subscribes** to the raw image topics from the Raspberry Pi.
    * **Processes** the images to extract hand keypoints and XYZ coordinates.
    * **Calculates** the Inverse Kinematics (IK) to determine the required servo angles.
    * **Publishes** the final angles back to the Raspberry Pi.

---

## 🔄 Data Flow (DDS Pipeline)

1.  **[RPI]** Camera Nodes ➔ `sensor_msgs/CompressedImage`
2.  **[PC]** Vision Node ➔ Hand Tracking ➔ IK Calculation
3.  **[PC]** Vision Node ➔ `std_msgs/Float64MultiArray` (Servo Angles)
4.  **[RPI]** Subscriber Node ➔ Serial/UART ➔ **ESP32**

---

## 🛠️ Installation & Setup

### **Build Instructions**
Navigate to the root of your workspace and run:
```bash
colcon build --symlink-install
source install/setup.bash

**Next Step:** For details on how the coordinates artificail system works, see [**Vision System**](../Artificial_System) documentation.