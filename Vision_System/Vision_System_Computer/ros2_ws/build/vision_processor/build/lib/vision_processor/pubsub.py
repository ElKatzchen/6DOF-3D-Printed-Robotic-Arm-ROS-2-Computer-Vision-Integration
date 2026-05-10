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
        self.current_signal = 1

        #----------ARM SIZE (mm)----------
        self.L1, self.L2, self.L3, self.L4 = 68.0, 165.0, 109.0, 157.0
        self.gravity_correction = 0.03

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
    #---Input are the two frames from the cameras---
    #---Output is the X Y Z coords and the Open/Close signal---
    def artificial_vision_system(self, img_u, img_s):
        xyz = list(self.last_xyz)
        signal = 1
        found_hand = False

        #----------SUPERIOR CAMERA PROCESSING (X, Y AND SIGNAL)----------
        if img_u is not None:
            f_u = cv2.undistort(img_u, self.calib_u['mtx'], self.calib_u['dist'], None, self.calib_u['mtx'])
            res_u = self.hands_u.process(cv2.cvtColor(f_u, cv2.COLOR_BGR2RGB))
            
            if res_u.multi_hand_landmarks:
                found_hand = True
                hand = res_u.multi_hand_landmarks[0]
                
                #----------PIXEL SIZES----------
                ix, iy = int(hand.landmark[8].x * 640), int(hand.landmark[8].y * 480)
                mx, my = int(hand.landmark[12].x * 640), int(hand.landmark[12].y * 480)
                
                #----------XYZ MAPPING----------
                xyz[0] = self.map_val(iy, self.X_BACK, self.X_FRONT)
                xyz[1] = self.map_val(ix, self.Y_LEFT, self.Y_RIGHT, centrar=True)
                
                distance = np.sqrt((ix - mx)**2 + (iy - my)**2)
                
                #----------PERIMETER LOGIC
                if distance < self.PIXELS_UMBRAL:
                    signal = 0
                    perimeter_colour = (0, 0, 255)
                else:
                    signal = 1
                    perimeter_colour = (0, 255, 0)
                
                #----------VISUALISATION----------
                cv2.circle(f_u, (ix, iy), self.PIXELS_UMBRAL, perimeter_colour, 2)
                cv2.circle(f_u, (ix, iy), 5, (0, 255, 0), -1)
                cv2.circle(f_u, (mx, my), 5, (255, 255, 0), -1)
                
                cv2.putText(f_u, f"Dist: {int(distance)}", (ix+10, iy-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, perimeter_colour, 1)

            cv2.imshow("SUPERIOR CAMERA", f_u)

        #----------LATERAL CAMERA PROCESSING (Z)----------
        if img_s is not None:
            f_s = cv2.undistort(img_s, self.calib_s['mtx'], self.calib_s['dist'], None, self.calib_s['mtx'])
            f_s_rot = cv2.rotate(f_s, cv2.ROTATE_90_COUNTERCLOCKWISE)
            res_s = self.hands_s.process(cv2.cvtColor(f_s_rot, cv2.COLOR_BGR2RGB))
            
            if res_s.multi_hand_landmarks:
                pt_z = res_s.multi_hand_landmarks[0].landmark[8]
                xyz[2] = self.map_val(pt_z.y * 640, self.Z_DOWN, self.Z_UP)
                cv2.circle(f_s_rot, (int(pt_z.x*480), int(pt_z.y*640)), 10, (255, 0, 255), -1)
            
            cv2.imshow("LATERAL CAMERA", f_s_rot)

        cv2.waitKey(1)
        return xyz, signal, found_hand
    
    #----------PREPROSSESING----------
    def preprossed_coords(self, x, y, z, signal):
        processed_x = int((x * 200) + 100)
        processed_y = int(y * 100)
        processed_z = int((-(z - 1) * 200) + 100)
        processed_xyz = [processed_x, processed_y, processed_z]
        if signal == 0:
            processed_signal = 90
        else:
            processed_signal = 150
        
        return processed_xyz, processed_signal

    #----------JOINT1 MIN MAX VALUES----------
    def clamp(self, n):
        return max(15, min(int(n), 165))
    
    #-----------INVERSE KINEMATICS SYSTEM----------
    def inverse_kinematics(self, x, y, z):
        angle_base = math.degrees(math.atan2(y, x))
        s6 = self.clamp(85 + angle_base)
        s3_val = 80 - (angle_base * 0.5)
        s3 = self.clamp(s3_val)
        j3_rad_offset = math.radians(s3_val - 80)
        r_total = math.sqrt(x**2 + y**2)
        r_proj = r_total / math.cos(j3_rad_offset) if math.cos(j3_rad_offset) != 0 else r_total
        h = z - self.L1
        s1 = self.clamp(90 + angle_base)
        total_reach = math.sqrt(r_proj**2 + h**2)

        if total_reach > (self.L2 + self.L3 + self.L4):
            return None

        for phi_deg in range(-90, 91, 5):
            phi = math.radians(phi_deg)
            r_w = r_proj - self.L4 * math.cos(phi)
            h_w = h - self.L4 * math.sin(phi)
            dist_w_sq = r_w**2 + h_w**2
            dist_w = math.sqrt(dist_w_sq)
            if dist_w > (self.L2 + self.L3) or dist_w < abs(self.L2 - self.L3):
                continue
            try:
                cos_elbow = (self.L2**2 + self.L3**2 - dist_w_sq) / (2 * self.L2 * self.L3)
                ang_elbow_int = math.degrees(math.acos(max(-1.0, min(1.0, cos_elbow))))
                ang_elev = math.atan2(h_w, r_w)
                cos_apert = (self.L2**2 + dist_w_sq - self.L3**2) / (2 * self.L2 * dist_w)
                ang_apert = math.acos(max(-1.0, min(1.0, cos_apert)))
                q_shoulder = math.degrees(ang_elev + ang_apert)
                offset = r_proj * self.gravity_correction
                s5 = self.clamp(80 + (90 - (q_shoulder + offset)))
                s4 = self.clamp(80 + (180 - ang_elbow_int))
                q_forearm = q_shoulder - (180 - ang_elbow_int)
                s2 = self.clamp(80 - (phi_deg - q_forearm))
                return [s1, s2, s3, s4, s5, s6]
            except:
                continue
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
            self.last_xyz = raw_xyz 

            processed_xyz, processed_signal = self.preprossed_coords(raw_xyz[0], raw_xyz[1], raw_xyz[2], raw_signal)
            self.current_signal = processed_signal

            res = self.inverse_kinematics(processed_xyz[0], processed_xyz[1], processed_xyz[2])

            if res is not None:
                msg = Int32MultiArray()
                msg.data = [int(processed_signal), int(res[0]), int(res[1]), int(res[2]), int(res[3]), int(res[4]), int(res[5])]
                
                self.angle_publisher.publish(msg)

                out = f"\rPUB -> SIG: {processed_signal} | S1: {res[0]} | S2: {res[1]} | S3: {res[2]} | S4: {res[3]} | S5: {res[4]} | S6: {res[5]}"
                sys.stdout.write(out)
                sys.stdout.flush()
            else:
                sys.stdout.write("\rOUT OF REACH - ARM CANNOT REACH COORDINATE")
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