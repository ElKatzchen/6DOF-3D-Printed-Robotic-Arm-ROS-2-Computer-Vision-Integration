import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pubsub')
        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            'angles',
            10)
        
        #----------ARM SIZE (mm)----------
        self.L1, self.L2, self.L3, self.L4 = 68.0, 165.0, 109.0, 157.0
        self.gravity_correction = 0.06

        #----------TIMER----------
        self.create_timer(0.1, self.coordinate_loop)
        self.get_logger().info("Node PUBSUB (IK) initialized. Witing coords...")

    #----------JOINT1 MIN MAX VALUES----------
    def clamp(self, n):
        return max(15, min(int(n), 165))

    #-----------INVERSE KINEMATICS LOGIC----------
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
                return [130, s1, s2, s3, s4, s5, s6]
            except:
                continue
        return None

    #----------LOOP----------
    def coordinate_loop(self):
        try:
            val = input("COORDS X,Y,Z (X TO EXIT): ")
            if val.lower() == 'x': return
            x, y, z = map(float, val.split(','))
            res = self.inverse_kinematics(x, y, z)
            
            if res:
                msg = Int32MultiArray()
                msg.data = res
                self.publisher_.publish(msg)
                self.get_logger().info(f"PUBLISHING ANGLES: {res}")
            else:
                self.get_logger().warn("OUT OF REACH POSITION")
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()
    rclpy.spin(node)
    rclpy.shutdown()