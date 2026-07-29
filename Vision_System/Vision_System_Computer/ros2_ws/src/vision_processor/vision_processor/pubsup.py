import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class PubSup(Node):
    def __init__(self):
        super().__init__('pubsup')
        self.publisher_ = self.create_publisher(CompressedImage, 'cam_sup', 1)
        
        #----------READ CAMERA----------
        self.cap = cv2.VideoCapture(6, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("PUBSUP NODE LAUNCHED")

    def timer_callback(self):
        if not self.cap.isOpened():
            return
            
        ret, frame = self.cap.read()
        if ret:
            try:
                success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 20])
                if success:
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = "jpeg"
                    msg.data = np.array(buffer).tobytes()
                    self.publisher_.publish(msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PubSup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()