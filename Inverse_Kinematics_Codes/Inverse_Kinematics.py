import math
import serial
import time
import numpy as np
from ikpy.chain import Chain

#----------CONFIGURACIÓN IKPY----------
# Asegúrate de que Arm.urdf esté en la misma carpeta que este script
chain = Chain.from_urdf_file("Arm.urdf", active_links_mask=[False, True, True, True, True, True, True])
limits = [
    [-1.309, 1.309], [-1.222, 1.222], [-1.309, 1.309], 
    [-1.222, 1.222], [-1.222, 1.222], [-1.309, 1.309]
]
target_vector = [1, 0, -1]

def calcular_cinematica(target_xyz, chain, limits, target_vector):
    try:
        # Cálculo de cinemática inversa
        angles = chain.inverse_kinematics(
            target_position=target_xyz, 
            target_orientation=target_vector, 
            orientation_mode="Z"
        )
        
        # Procesamiento de ángulos
        raw_angles = angles[1:]
        restricted_angles = [np.clip(raw_angles[i], limits[i][0], limits[i][1]) for i in range(len(raw_angles))]
        
        # Inversión de orden (de punta a base)
        inverted_angles = restricted_angles[::-1]
        
        # Conversión a grados y aplicación de offsets
        degrees = np.degrees(inverted_angles)
        offsets = np.array([85, 80, 80, 80, 75, 80])
        result = degrees + offsets
        
        # Retorno como enteros
        return result.astype(int)
    except Exception as e:
        print(f"CALCULATION ERROR: {e}")
        return None

#-----LOOP PRINCIPAL-----
try:
    esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    time.sleep(3)
    print("SYSTEM READY (IKPY MODE)")

    while True:
        input_coords = input("\nCOORDS X, Y, Z (Enteros): ")
        if input_coords.lower() == 'x': break
        try:
            parts = [v.strip() for v in input_coords.split(',')]
            # Convertimos las coordenadas directamente a enteros
            x, y, z = map(int, parts)
            
            # Llamada a la función de cinemática
            res = calcular_cinematica([x, y, z], chain, limits, target_vector)
            
            if res is not None:
                # Formato de envío: [90, Servo1, Servo2, Servo3, Servo4, Servo5, Servo6]
                full_vals = [150] + list(res) 
                payload = "$" + "/".join([f"{int(v):03d}" for v in full_vals]) + "\n"
                
                esp32.reset_input_buffer()
                esp32.write(payload.encode())
                print(f"SEND -> {payload.strip()}")

                time.sleep(0.05)
                if esp32.in_waiting > 0:
                    feedback = esp32.readline().decode('utf-8').strip()
                    print(f"FEEDBACK -> {feedback}")
            else:
                print("ALERT: IMPOSSIBLE POSITION")
        except Exception as e:
            print(f"ERROR: {e}")
finally:
    if 'esp32' in locals(): 
        esp32.close()