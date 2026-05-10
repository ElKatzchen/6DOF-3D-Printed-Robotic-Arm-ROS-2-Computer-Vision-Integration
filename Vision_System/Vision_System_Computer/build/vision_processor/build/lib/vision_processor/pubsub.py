import sys
import os

# --- FILTRO DE SEGURIDAD (ANTIBOMBAS) ---
# Esto elimina la interferencia de la carpeta .local y fuerza a usar Conda
sys.path = [p for p in sys.path if ".local" not in p]

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np

# Intentamos importar mediapipe después de limpiar el path
try:
    import mediapipe as mp
    # Verificación silenciosa para el log
    mp_path = mp.__file__
except AttributeError:
    # Si por alguna razón extrema sigue fallando, forzamos el sub-import
    import mediapipe.python.solutions.hands as mp_hands
    import mediapipe.python.solutions.drawing_utils as mp_drawing
    mp.solutions = type('obj', (object,), {'hands': mp_hands, 'drawing_utils': mp_drawing})

class StereoProcessorNode(Node):
    def __init__(self):
        super().__init__('stereo_processor_node')
        
        # Log de diagnóstico para tu tranquilidad
        self.get_logger().info(f"✅ MediaPipe cargado desde: {mp.__file__}")

        # --- CARGA DE CALIBRACIÓN ---
        try:
            package_share_directory = get_package_share_directory('vision_processor')
            path_u = os.path.join(package_share_directory, 'calib_upside_data.npz')
            path_s = os.path.join(package_share_directory, 'calib_side_data.npz')
            
            self.calib_u = np.load(path_u)
            self.calib_s = np.load(path_s)
            self.get_logger().info("✅ Calibración .npz cargada correctamente.")
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando archivos de calibración: {e}")
            raise e

        # --- PARÁMETROS DEL ESPACIO (33x33x33 cm) ---
        self.X_ATRAS, self.X_FRENTE = 480, 15
        self.Y_IZQ, self.Y_DER = 561, 66
        self.Z_SUELO, self.Z_TECHO = 588, 118

        # --- CONFIGURACIÓN DE MEDIAPIPE ---
        # Usamos mp.solutions que ahora debería estar disponible
        self.mp_hands = mp.solutions.hands
        self.hands_u = self.mp_hands.Hands(min_detection_confidence=0.7, max_num_hands=1)
        self.hands_s = self.mp_hands.Hands(min_detection_confidence=0.7, max_num_hands=1)

        # --- MEMORIA DE ESTADO ---
        self.frame_u = None
        self.frame_s = None
        self.last_xyz = [0.5, 0.0, 0.2] 

        # --- SUBSCRIPTORES ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CompressedImage, '/cam_sup', self.callback_u, qos)
        self.create_subscription(CompressedImage, '/cam_side', self.callback_s, qos)

        # --- TIMER DE PROCESAMIENTO (20 Hz) ---
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🚀 Sistema XYZ operativo y filtrado.")

    def map_val(self, val, l1, l2, centrar=False):
        p = np.clip((val - l1) / (l2 - l1), 0, 1)
        return (p - 0.5) * 2 if centrar else p

    def procesar_xyz(self, img_u, img_s):
        xyz = list(self.last_xyz)

        # Cámara Superior (X, Y)
        if img_u is not None:
            f_u = cv2.undistort(img_u, self.calib_u['mtx'], self.calib_u['dist'], None, self.calib_u['mtx'])
            res_u = self.hands_u.process(cv2.cvtColor(f_u, cv2.COLOR_BGR2RGB))
            if res_u.multi_hand_landmarks:
                pt = res_u.multi_hand_landmarks[0].landmark[8]
                xyz[0] = self.map_val(pt.y * 480, self.X_ATRAS, self.X_FRENTE)
                xyz[1] = self.map_val(pt.x * 640, self.Y_DER, self.Y_IZQ, centrar=True)
                cv2.circle(f_u, (int(pt.x*640), int(pt.y*480)), 10, (0, 255, 0), -1)
            cv2.imshow("Vista Superior (X-Y)", f_u)

        # Cámara Lateral (Z)
        if img_s is not None:
            f_s = cv2.undistort(img_s, self.calib_s['mtx'], self.calib_s['dist'], None, self.calib_s['mtx'])
            f_s_rot = cv2.rotate(f_s, cv2.ROTATE_90_COUNTERCLOCKWISE)
            res_s = self.hands_s.process(cv2.cvtColor(f_s_rot, cv2.COLOR_BGR2RGB))
            if res_s.multi_hand_landmarks:
                pt = res_s.multi_hand_landmarks[0].landmark[8]
                xyz[2] = self.map_val(pt.y * 640, self.Z_SUELO, self.Z_TECHO)
                cv2.circle(f_s_rot, (int(pt.x*480), int(pt.y*640)), 10, (255, 0, 255), -1)
            cv2.imshow("Vista Lateral (Z)", f_s_rot)

        cv2.waitKey(1)
        return xyz

    def callback_u(self, msg):
        self.frame_u = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    def callback_s(self, msg):
        self.frame_s = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    def control_loop(self):
        if self.frame_u is not None or self.frame_s is not None:
            self.last_xyz = self.procesar_xyz(self.frame_u, self.frame_s)
            # Log más limpio para no saturar
            sys.stdout.write(f"\rX: {self.last_xyz[0]:.2f} | Y: {self.last_xyz[1]:.2f} | Z: {self.last_xyz[2]:.2f}")
            sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = StereoProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()