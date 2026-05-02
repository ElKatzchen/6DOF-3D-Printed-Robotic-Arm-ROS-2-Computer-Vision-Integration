import math
import serial
import time
import numpy as np

#----------SIZES (mm)----------
L1, L2, L3, L4 = 68.0, 165.0, 109.0, 157.0

#----------DENAVIT HARTEMBERG MATRIX----------
def matrix_dh(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

#----------INVERSE KINEMATICS----------
def inverse_kinematics(x, y, z):
    #----------BASE AND HORIZONTAL ORIENTATION (JOINT6, JOINT5, JOINT4)----------
    theta_base = np.arctan2(y, x)
    angle_base_deg = np.degrees(theta_base)
    
    s1 = int(np.clip(90 + angle_base_deg, 15, 165))
    s6 = int(np.clip(85 + angle_base_deg, 15, 165))
    s3_val = 80 - (angle_base_deg * 0.5)
    s3 = int(np.clip(s3_val, 15, 165))
    
    j3_offset = np.radians(s3_val - 80)

    r_total = np.sqrt(x**2 + y**2)
    r_proj = r_total / np.cos(j3_offset) if np.cos(j3_offset) != 0 else r_total
    h = z - L1

    #----------POSTURE SEARCH----------
    for phi_deg in np.arange(-110, 111, 1):
        phi_rad = np.radians(phi_deg)

        #----------CENTRAL LOCALIZATION OF THE GRIPPER----------
        T_target = np.array([
            [np.cos(phi_rad), -np.sin(phi_rad), 0, r_proj],
            [np.sin(phi_rad),  np.cos(phi_rad), 0, h],
            [0,               0,                1, 0],
            [0,               0,                0, 1]
        ])
        
        T_tool = matrix_dh(0, 0, L4, 0)
        T_wrist = np.dot(T_target, np.linalg.inv(T_tool))
        
        r_w = T_wrist[0, 3]
        h_w = T_wrist[1, 3]
        
        dist_sq = r_w**2 + h_w**2
        dist = np.sqrt(dist_sq)

        if dist > (L2 + L3) or dist < abs(L2 - L3):
            continue

        try:
            #----------ANGLE SOLUTION DH----------
            #----------THETA3 ELBOW (JOINT4)----------
            cos_t3 = (dist_sq - L2**2 - L3**2) / (2 * L2 * L3)
            t3 = np.arccos(np.clip(cos_t3, -1.0, 1.0))
            
            s4_val = 80 + (180 - np.degrees(t3))
            
            if s4_val < 5 or s4_val > 165:
                continue

            #----------THETA2 SHOULDER (JOINT5)----------
            t2 = np.arctan2(h_w, r_w) + np.arccos(np.clip((L2**2 + dist_sq - L3**2) / (2 * L2 * dist), -1.0, 1.0))

            #----------DIRECT CINEMATIC VERIFICATION WITH DH----------
            T12 = matrix_dh(t2, 0, L2, 0)
            T23 = matrix_dh(-t3, 0, L3, 0)
            T_arm = np.dot(T12, T23)
            
            #----------ERROR RESTRICTOR----------
            if abs(T_arm[0, 3] - r_w) > 5.0 or abs(T_arm[1, 3] - h_w) > 5.0:
                continue

            gravity_comp = r_proj * 0.06
            
            s5 = int(np.clip(80 + (90 - (np.degrees(t2) + gravity_comp)), 5, 175))
            s4 = int(s4_val)
            
            #----------WRIST ANGLE----------
            q_forearm = np.degrees(t2) - (180 - np.degrees(t3))
            s2 = int(np.clip(80 - (phi_deg - q_forearm), 10, 170))

            return [s1, s2, s3, s4, s5, s6]
            
        except:
            continue

    return "IMPOSSIBLE POSITION"

#-----LOOP-----
try:
    #----------ESP SENDER INFO----------
    esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    time.sleep(3)
    print("SYSTEM READY")

    while True:
        input_coords = input("\nCOORDS X, Y, Z: ")
        if input_coords.lower() == 'x': break
        try:
            parts = [v.strip() for v in input_coords.split(',')]
            if len(parts) != 3:
                print("ERROR. EXPECTED FORMAT: X, Y, Z")
                continue
            
            x, y, z = map(float, parts)
            res = inverse_kinematics(x, y, z)
            
            if isinstance(res, list):
                #----------SENT 2 TRASH DATA----------
                full_vals = [0, 0, 90] + res 
                payload = "$" + "/".join([f"{int(v):03d}" for v in full_vals]) + "\n"
                
                esp32.reset_input_buffer()
                esp32.write(payload.encode())
                esp32.flush()
                print(f"SEND -> {payload.strip()}")

                time.sleep(0.05)
                if esp32.in_waiting > 0:
                    feedback = esp32.readline().decode('utf-8').strip()
                    print(f"FEEDBACK -> {feedback}")
            else:
                print(f"ALERT: {res}")
        except Exception as e:
            print(f"ERROR: {e}")
finally:
    if 'esp32' in locals(): 
        esp32.close()