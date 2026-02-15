# 🏗️ Hardware & CAD Models

This section contains all the **SolidWorks 2025** source files and **STL** files required to fabricate the 6DOF robotic arm and the computer vision frame.

## 📂 Folder Structure

The hardware files are organized into three main categories to facilitate navigation:

1. **Vision Frame:** Contains the 3 components required to assemble the physical structure for the dual-camera setup.
2. **Arm - Version A (SG90S + MG996R):** Optimized for standard SG90S micro servos and MG996R metal gear motors.
3. **Arm - Version B (ES08MA II + MG996R):** Optimized for Emax ES08MA II high-torque micro servos and MG996R motors.

> **Note:** If you do not wish to modify the designs, pre-exported files are available in the **STLs** folder within each category for direct 3D printing.

---

## 🔩 Technical Specifications

### **Hardware & Fasteners**
To ensure structural integrity, the design uses specific bolt sizes:
* **Main Joints:** Designed for **M2 bolts**.
* **Base Assembly:** Designed for **M6 bolts** to provide a stable foundation.

### **Component Naming Convention**
Files are prefixed based on their function to simplify the assembly process:
* `Claw_`: End effector and gripper components.
* `Arm_`: Main structural segments and linkages.
* `Base_`: Rotating foundation and main axis.
* `Holder_`: Reinforced supports for critical sections and motor mounting.

### **Actuators (Motors)**
To fully assemble one arm, you will need the following servos:
* **4x** Micro Servos (either **SG90S** or **ES08MA II**, depending on your chosen version).
* **3x** **MG996R** High-Torque Metal Gear Servos.

### **Vision Area**
* The vision frame is precisely calibrated to delimit a workspace of **$33 \times 33 \times 33$ cm**.

---

## 🖨️ 3D Printing Recommendations

For best results and mechanical strength, the following settings are suggested:

* **Material:** PLA+ or PETG (for better impact resistance).
* **Infill:** 5-15% (Gyroid pattern) for arm segments.
* **Wall Count:** Minimum of 3-4 wall lines for all load-bearing parts.
* **Supports & Bed Adhesion:**
    * **Full Support Required:** It is highly recommended to use supports for all parts.
    * **3mm Offset:** All pieces should be printed with a **3mm support base/offset from the heat bed**. This prevents bottom-layer deformation (elephant's foot) and ensures that all mechanical dimensions remain accurate and consistent across the entire part.
    
---

