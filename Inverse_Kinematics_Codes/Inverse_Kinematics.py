import math
import serial
import time

#----------SIZES (mm)----------
L1, L2, L3, L4 = 68.0, 165.0, 109.0, 157.0

#----------GRAVITY COMPENSATOR----------
gravity_correction_constant = 0.06 

#----------JOINT1 SECURITY ANGLE----------
def clamp(n):
    return max(15, min(int(n), 165))

#----------INVERSE KINEMATICS FUNCTION----------
def inverse_kinematics(x, y, z):
    #----------JOINT6 HORIZONTAL ROTATION----------
    angle_base = math.degrees(math.atan2(y, x))
    s6 = clamp(85 + angle_base)

    #----------JOINT3 AND ITS REACH----------
    s3_val = 80 - (angle_base * 0.5)
    s3 = clamp(s3_val)
    j3_rad_offset = math.radians(s3_val - 80)

    #---------- CORRECTED VERTICAL GEOMETRY----------
    r_total = math.sqrt(x**2 + y**2)
    r_proj = r_total / math.cos(j3_rad_offset) if math.cos(j3_rad_offset) != 0 else r_total
    h = z - L1
    
    #----------JOINT1 YAW PITCH ROLL----------
    s1 = clamp(90 + angle_base)

    #----------JOINT2, JOINT4 AND JOINT5 POSTURE CALCULATION----------
    total_reach = math.sqrt(r_proj**2 + h**2)
    if total_reach > (L2 + L3 + L4):
        return f"OUT OF REACH: {total_reach:.1f}mm"

    #----------ATTACH ANGLE PHI----------
    for phi_deg in range(-90, 91, 5):
        phi = math.radians(phi_deg)
        
        #----------WRIST POSITION----------
        r_w = r_proj - L4 * math.cos(phi)
        h_w = h - L4 * math.sin(phi)
        
        dist_w_sq = r_w**2 + h_w**2
        dist_w = math.sqrt(dist_w_sq)

        #----------WRIST REACH VERIFICATION----------
        if dist_w > (L2 + L3) or dist_w < abs(L2 - L3):
            continue

        try:
            #----------JOINT4 ELBOW COS THEOREM ANGLE----------
            cos_elbow = (L2**2 + L3**2 - dist_w_sq) / (2 * L2 * L3)
            ang_elbow_int = math.degrees(math.acos(max(-1.0, min(1.0, cos_elbow))))
            
            #----------JOINT5 SHOULDER ANGLE
            ang_elev = math.atan2(h_w, r_w)
            cos_apert = (L2**2 + dist_w_sq - L3**2) / (2 * L2 * dist_w)
            ang_apert = math.acos(max(-1.0, min(1.0, cos_apert)))
            
            q_shoulder = math.degrees(ang_elev + ang_apert)

            #----------GRAVITY COMPENSATION----------
            offset = r_proj * gravity_correction_constant
            
            s5 = clamp(80 + (90 - (q_shoulder + offset)))
            s4 = clamp(80 + (180 - ang_elbow_int))
            
            #----------JOINT2 WRIST ANGLE (PITCH)----------
            q_forearm = q_shoulder - (180 - ang_elbow_int)
            s2 = clamp(80 - (phi_deg - q_forearm))

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
                full_vals = [0, 0, 150] + res 
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