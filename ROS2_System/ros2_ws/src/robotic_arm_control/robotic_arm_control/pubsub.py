import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from ikpy.chain import Chain
import numpy as np

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        
        self.chain = Chain.from_urdf_file("Arm.urdf", active_links_mask=[False, True, True, True, True, True, True])
        self.limits = [[-1.309, 1.309], [-1.222, 1.222], [-1.309, 1.309], [-1.222, 1.222], [-1.222, 1.222], [-1.309, 1.309]]
        self.target_vector = [1, 0, -1]
        
        self.angle_publisher = self.create_publisher(Int32MultiArray, 'angles', 10)
        self.get_logger().info("MANUAL CONTROL LAUNCHED. FORMAT: X,Y,Z,O/C (e.g. 0.3,0.0,0.2,O)")

    def inverse_kinematics(self, target_xyz):
        try:
            angles = self.chain.inverse_kinematics(
                target_position=target_xyz, 
                target_orientation=self.target_vector, 
                orientation_mode="Z"
            )
            raw_angles = angles[1:]
            restricted_angles = [np.clip(raw_angles[i], self.limits[i][0], self.limits[i][1]) for i in range(len(raw_angles))]
            inverted_angles = restricted_angles[::-1]
            degrees = np.degrees(inverted_angles)
            offsets = np.array([85, 80, 80, 80, 75, 80])
            result = degrees + offsets
            result[0] = (offsets[2] + offsets[0]) - result[2]
            return result.astype(int)
        except Exception:
            return None

    def start_manual_loop(self):
        while rclpy.ok():
            user_input = input("Enter X,Y,Z,State (O=30, C=90): ")
            try:
                parts = user_input.split(',')
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                state = parts[3].strip().upper()
                
                signal = 30 if state == 'O' else 90
                
                res = self.inverse_kinematics([x, y, z])
                
                if res is not None:
                    msg = Int32MultiArray()
                    msg.data = [signal, int(res[0]), int(res[1]), int(res[2]), int(res[3]), int(res[4]), int(res[5])]
                    self.angle_publisher.publish(msg)
                    print(f"PUBLISHED: {msg.data}")
            except Exception as e:
                print(f"ERROR: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        node.start_manual_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()