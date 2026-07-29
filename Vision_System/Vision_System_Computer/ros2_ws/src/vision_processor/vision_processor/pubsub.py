import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import math
from ikpy.chain import Chain

#----------CONDA/ROS FILTER----------
sys.path = [p for p in sys.path if ".local" not in p]

#----------SPECIAL IMPORT FOR MEDIAPIPE----------
try:
    import mediapipe as mp
except AttributeError:
    import mediapipe.python.solutions.hands as mp_hands
    import mediapipe.python.solutions.drawing_utils as mp_drawing
    mp.solutions = type('obj', (object,), {'hands': mp_hands, 'drawing_utils': mp_drawing})

class StereoProcessorNode(Node):
    def __init__(self):
        super().__init__('stereo_processor_node')
        
        #----------CALIBRATION LOAD----------
        try:
            package_share_directory = get_package_share_directory('vision_processor')
            path_u = os.path.join(package_share_directory, 'calib_upside_data.npz')
            path_s = os.path.join(package_share_directory, 'calib_side_data.npz')
            
            self.calib_u = np.load(path_u)
            self.calib_s = np.load(path_s)
            self.get_logger().info("CALIBRATION DATA LOADED")
        except Exception as e:
            self.get_logger().error(f"ERROR DURING CALIBRATION: {e}")
            raise e

        #----------CUBE PARAMETERS----------
        self.X_BACK, self.X_FRONT = 480, 15
        self.Y_LEFT, self.Y_RIGHT = 561, 66
        self.Z_UP, self.Z_DOWN = 588, 118

        #----------AREA PIXELS----------
        self.PIXELS_UMBRAL = 45 

        #----------MEDIAPIPE CONFIGURATION----------
        self.mp_hands = mp.solutions.hands
        self.hands_u = self.mp_hands.Hands(min_detection_confidence=0.8, max_num_hands=1)
        self.hands_s = self.mp_hands.Hands(min_detection_confidence=0.8, max_num_hands=1)

        #----------MEMORY----------
        self.frame_u = None
        self.frame_s = None
        self.last_xyz = [0.5, 0.0, 0.2] 

        self.last_round_x = 100
        self.last_round_y = 0
        self.last_round_z = 100

        #----------ARM SIZE (mm)----------
        self.chain = Chain.from_urdf_file("Arm.urdf", active_links_mask=[False, True, True, True, True, True, True])
        self.limits = [[-1.309, 1.309], [-1.222, 1.222], [-1.309, 1.309], [-1.222, 1.222], [-1.222, 1.222], [-1.309, 1.309]]
        self.target_vector = [1, 0, -1]

        #----------KALMAN FILTER PARAMS----------
        self.Q = 0.1
        self.R = 0.0001
        self.P = 1.0
        self.kalman_x = 0.5
        self.kalman_y = 0.0
        self.kalman_z = 0.2

        #----------ROS2 CONFIG----------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CompressedImage, '/cam_sup', self.callback_u, qos)
        self.create_subscription(CompressedImage, '/cam_side', self.callback_s, qos)

        self.angle_publisher = self.create_publisher(Int32MultiArray, 'angles', 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("PUBSUB NODE LAUNCHED")

    def map_val(self, val, l1, l2, centrar=False):
        p = np.clip((val - l1) / (l2 - l1), 0, 1)
        return (p - 0.5) * 2 if centrar else p

    #----------ARTIFICIAL VISION SYSTEM----------
    def artificial_vision_system(self, img_u, img_s):
        xyz = list(self.last_xyz)
        signal_target = 90.0
        found_hand = False

        if img_u is not None:
            f_u = cv2.undistort(img_u, self.calib_u['mtx'], self.calib_u['dist'], None, self.calib_u['mtx'])
            res_u = self.hands_u.process(cv2.cvtColor(f_u, cv2.COLOR_BGR2RGB))
            
            if res_u.multi_hand_landmarks:
                found_hand = True
                hand = res_u.multi_hand_landmarks[0]
                
                ix, iy = int(hand.landmark[8].x * 640), int(hand.landmark[8].y * 480)
                mx, my = int(hand.landmark[12].x * 640), int(hand.landmark[12].y * 480)
                
                xyz[0] = self.map_val(iy, self.X_BACK, self.X_FRONT)
                xyz[1] = self.map_val(ix, self.Y_LEFT, self.Y_RIGHT, centrar=True)
                
                distance = np.sqrt((ix - mx)**2 + (iy - my)**2)
                signal_target = 30.0 if distance < self.PIXELS_UMBRAL else 80.0
                
                cv2.circle(f_u, (ix, iy), self.PIXELS_UMBRAL, (0, 255, 0), 2)
            cv2.imshow("SUPERIOR CAMERA", f_u)

        if img_s is not None:
            f_s = cv2.undistort(img_s, self.calib_s['mtx'], self.calib_s['dist'], None, self.calib_s['mtx'])
            f_s_rot = cv2.rotate(f_s, cv2.ROTATE_90_COUNTERCLOCKWISE)
            res_s = self.hands_s.process(cv2.cvtColor(f_s_rot, cv2.COLOR_BGR2RGB))
            
            if res_s.multi_hand_landmarks:
                pt_z = res_s.multi_hand_landmarks[0].landmark[8]
                xyz[2] = self.map_val(pt_z.y * 640, self.Z_DOWN, self.Z_UP)
            cv2.imshow("LATERAL CAMERA", f_s_rot)

        cv2.waitKey(1)
        return xyz, signal_target, found_hand
    
    #----------HISTERESIS----------
    def custom_round(self, val, last_valid_val):
        last_digit = abs(val) % 10
        base = (abs(val) // 10) * 10
        
        if last_digit in [1, 2, 3]:
            result = base
        elif last_digit in [7, 8, 9]:
            result = base + 10
        else:
            return last_valid_val
            
        final_result = result if val >= 0 else -result
        return final_result
    
    #----------PREPROSSESING----------
    def preprossed_coords(self, x, y, z):
        x_clipped = max(0.3, min(x, 1.0))
        z_clipped = max(0.0, min(z, 0.9))
        y_clipped = max(-0.9, min(y, 0.9))

        raw_x = int(((x_clipped / 1.0) * 200) + 100)
        raw_y = int(-((y_clipped / 0.9) * 100))
        raw_z = int((((-(z_clipped / 0.9) * 300)) + 300) - 20)

        self.last_round_x = self.custom_round(raw_x, self.last_round_x)
        self.last_round_y = self.custom_round(raw_y, self.last_round_y)
        self.last_round_z = self.custom_round(raw_z, self.last_round_z)
        
        return [self.last_round_x, self.last_round_y, self.last_round_z]
    
    def apply_kalman(self, measurement, state):
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        state = state + K * (measurement - state)
        self.P = (1 - K) * self.P
        return state

    #-----------INVERSE KINEMATICS SYSTEM----------
    def inverse_kinematics(self, target_xyz, target_vector, chain, limits):
        try:
            angles = chain.inverse_kinematics(
                target_position=target_xyz, 
                target_orientation=target_vector, 
                orientation_mode="Z"
            )

            raw_angles = angles[1:]
            
            restricted_angles = []
            for i in range(len(raw_angles)):
                min_lim, max_lim = limits[i]
                val = np.clip(raw_angles[i], min_lim, max_lim)
                restricted_angles.append(val)

            inverted_angles = restricted_angles[::-1]
            degrees = np.degrees(inverted_angles)
            offsets = np.array([85, 80, 80, 80, 75, 80])
            result = degrees + offsets
            result[0] = (offsets[2] + offsets[0]) - result[2]
            
            return result.astype(int)
        except Exception:
            return None

    #----------CAMERA READING----------
    def callback_u(self, msg):
        self.frame_u = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    def callback_s(self, msg):
        self.frame_s = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    #----------MAIN LOOP----------
    def control_loop(self):
        if self.frame_u is not None or self.frame_s is not None:
            raw_xyz, raw_signal, detected = self.artificial_vision_system(self.frame_u, self.frame_s)
            if not detected:
                return
            
            self.kalman_x = self.apply_kalman(raw_xyz[0], self.kalman_x)
            self.kalman_y = self.apply_kalman(raw_xyz[1], self.kalman_y)
            self.kalman_z = self.apply_kalman(raw_xyz[2], self.kalman_z)

            processed_xyz = self.preprossed_coords(self.kalman_x, self.kalman_y, self.kalman_z)
            processed_signal = int(raw_signal)

            res = self.inverse_kinematics(processed_xyz, self.target_vector, self.chain, self.limits)

            if res is not None:
                msg = Int32MultiArray()
                msg.data = [processed_signal, int(res[0]), int(res[1]), int(res[2]), int(res[3]), int(res[4]), int(res[5])]
                self.angle_publisher.publish(msg)

                out = (
                    f"\rPUB -> SIG: {processed_signal} | S1: {res[0]} | S2: {res[1]} | S3: {res[2]} | S4: {res[3]} | S5: {res[4]} | S6: {res[5]}\033[K\n"
                    f"COORDS -> X: {processed_xyz[0]} | Y: {processed_xyz[1]} | Z: {processed_xyz[2]}\033[K\n"
                    f"\033[2A"
                )
                sys.stdout.write(out)
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