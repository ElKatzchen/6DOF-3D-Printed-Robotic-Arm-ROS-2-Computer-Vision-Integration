import math
import serial
import time

#-----SIZES-----
L1, L2, L3, L4 = 68.0, 165.0, 109.0, 157.0

#-----GRAVITY COMPENSATION-----
gravity_correction_constant = 0.06 

#-----INVERSE KINEMATICS-----
def inverse_kinematics(x, y, z):
    #-----HORIZONTAL ROTATION (S3 and S6)-----
    total_angle_objective = math.degrees(math.atan2(y, x))
    s6 = 85 + (total_angle_objective * 0.7)
    s3 = 80 + (total_angle_objective * 0.3)

    #-----VERTICAL GEOMETRY-----
    r = math.sqrt(x**2 + y**2)
    h = z - L1
    total_reach = math.sqrt(r**2 + h**2)

    if total_reach > (L2 + L3 + L4):
        return f"EXTENDED REACH: {total_reach:.1f}mm"

    #-----VERSICAL POSTURE-----
    for phi_deg in range(-90, 91, 5):
        phi = math.radians(phi_deg)
        r_w = r - L4 * math.cos(phi)
        h_w = h - L4 * math.sin(phi)
        dist_w_sq = r_w**2 + h_w**2
        dist_w = math.sqrt(dist_w_sq)

        if dist_w > (L2 + L3) or dist_w < abs(L2 - L3):
            continue

        try:
            cos_elbow = (L2**2 + L3**2 - dist_w_sq) / (2 * L2 * L3)
            ang_elbow_int = math.degrees(math.acos(max(-1, min(1, cos_elbow))))
            
            ang_elev = math.atan2(h_w, r_w)
            cos_apert = (L2**2 + dist_w_sq - L3**2) / (2 * L2 * dist_w)
            ang_apert = math.acos(max(-1, min(1, cos_apert)))
            
            q_shoulder = math.degrees(ang_elev + ang_apert)

            #-----DINAMIC CORRECTION-----
            offset = r * gravity_correction_constant
            
            s5 = 80 + (90 - (q_shoulder + offset))
            s4 = 80 + (180 - ang_elbow_int)
            
            q_forearm = q_shoulder - (180 - ang_elbow_int)
            vertical_error = phi_deg - q_forearm
            s2 = 80 - vertical_error 

            s1 = 90 

            if all(0 <= v <= 180 for v in [s1, s2, s3, s4, s5, s6]):
                return [int(s1), int(s2), int(s3), int(s4), int(s5), int(s6)]
        except:
            continue

    return "IMPOSIBLE POSITION"

#-----LOOP-----
try:
    esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)
    print(f"COMPENSATION ACTIVE OF ({gravity_correction_constant})")
    print("INTRODUCE X TO EXIT")

    while True:
        input = input("COORDS X, Y, Z: ")
        if input.lower() == 'x': break
        try:
            x, y, z = [float(v.strip()) for v in input.split(',')]
            res = inverse_kinematics(x, y, z)
            if isinstance(res, list):
                s1, s2, s3, s4, s5, s6 = res
                payload = f"150,{s1},{s2},{s3},{s4},{s5},{s6}\n"
                esp32.write(payload.encode())
                print(f"OK -> SENDING CORRECTED S5: {s5}")
            else:
                print(f"ALERT: {res}")
        except:
            print("FORMAT: X, Y, Z")
finally:
    if 'esp32' in locals(): esp32.close()