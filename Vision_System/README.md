# 👁️ Computer Vision & Spatial Mapping System

This folder contains the core logic for dual-camera hand tracking, coordinate mapping, and the distributed ROS 2 pipeline. It is divided into three specialized submodules.

---

## 🛠️ 1. Camera Calibration (`/calibration`)
Before operation, the cameras must be intrinsically calibrated to ensure spatial accuracy.
* **Process:** Take 25 photos of a chessboard pattern from different angles.
* **Default Pattern:** 10x7 squares, with a square size of **26mm** (adjustable in the script).
* **Output:** Generates `.npz` files containing the camera matrices and distortion coefficients.

---

## 🖥️ 2. Workstation Logic (`/Vision_System_Computer`)
This is the central processing unit of the vision system. It runs a **Publisher/Subscriber** node that performs the following:

1. **Data Acquisition:** Subscribes to `/cam_sup` and `/cam_side` image topics.
2. **Keypoint Detection:** Uses an AI-based system to detect hand keypoints:
    * **Index Finger:** Determines the target **$XYZ$ coordinates**.
    * **Middle Finger:** Acts as a trigger for the **gripper** (open/close logic).
3. **Processing Pipeline:** The coordinates are pre-processed and fed into the **Inverse Kinematics (IK)** function to calculate the 7 required servo angles.
4. **Broadcast:** Sends the final joint array to the subscriber node.

---

## 🍓 3. Edge Processing (`/Vision_System_RPI`)
Managed via the Raspberry Pi, this module handles hardware-level communication.

* **Nodes:**
    * **2x Publishers:** One for each camera stream.
    * **1x Subscriber:** Receives 7 angles at a frequency of **5Hz** (5 times/sec) to filter noise and ensure stable movement.
* **Hardware Interface:** Forwards the validated data via **UART** to the ESP32.

### 🐳 Docker Deployment (RPI)
The RPI environment is containerized for consistency. To deploy:

```bash
docker compose down
docker compose up -d
docker compose exec ros2_env bash
```

Once inside the Docker bash terminal, initialize the workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
### 🌐 Network Configuration
To enable communication between the Computer and the RPI, set the ROS_DOMAIN_ID on both devices:
```bash
export ROS_DOMAIN_ID=0
echo $ROS_DOMAIN_ID
```

Additionally, on the Raspberry Pi terminal, export its IP address (must match your mobile hotspot IP):
```bash
export ROS_IP=192.168.X.X
```

#### 📦 Dependencies & Troubleshooting
1. **Serial Communication:** Install the necessary library for UART inside the Docker container:
    ```bash
    apt-get update && apt-get install -y python3-serial
    ```
2. **MediaPipe & Conda Integration:** If you encounter compatibility issues between MediaPipe and ROS 2, run ROS 2 within your Conda environment and manually link the site-packages:
    ```bash
    export PYTHONPATH=$PYTHONPATH:/home/katzchen/miniconda3/envs/taller/lib/python3.10/site-packages
    ```
3. **Hardware Verification:** To verify camera detection on the RPI, use:
    ```bash
    v4l2-ctl --list-devices
    ```