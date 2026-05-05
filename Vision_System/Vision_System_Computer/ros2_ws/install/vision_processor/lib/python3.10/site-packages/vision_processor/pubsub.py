import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('pc_visualizer_node')
        
        self.sub_sup = self.create_subscription(
            Image, 
            'cam_sup', 
            self.callback_sup, 
            10)
            
        self.sub_side = self.create_subscription(
            Image, 
            'cam_side', 
            self.callback_side, 
            10)
            
        self.bridge = CvBridge()
        self.get_logger().info('WAITING IMAGES')

    def callback_sup(self, msg):
        try:
            frame_sup = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("SUPERIOR VIEW", frame_sup)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'ERROR IN SUPERIOR: {e}')

    def callback_side(self, msg):
        try:
            frame_side = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("LATERAL VIEW", frame_side)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'ERROR IN SIDE: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
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