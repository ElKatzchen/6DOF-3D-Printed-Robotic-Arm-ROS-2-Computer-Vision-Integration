import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

class SubNode(Node):
    def __init__(self):
        super().__init__('sub')
        try:
            self.esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info("Connection UART with ESP32 established.")
        except Exception as e:
            self.get_logger().error(f"ERROR: {e}")

        self.last_angles = None
        self.last_send_time = time.time()
        self.send_interval = 0.2 
        self.threshold = 2       

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'angles',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        current_time = time.time()
        new_angles = list(msg.data)

        if (current_time - self.last_send_time) < self.send_interval:
            return

        if self.last_angles is not None:
            significant_change = any(abs(n - l) > self.threshold for n, l in zip(new_angles, self.last_angles))
            if not significant_change:
                return

        payload = "$" + "/".join([f"{int(v):03d}" for v in new_angles]) + "\n"
        
        try:
            self.esp32.write(payload.encode())
            self.esp32.flush()
            
            self.last_angles = new_angles
            self.last_send_time = current_time
            
            self.get_logger().info(f"SENDING: {payload.strip()}")
            
            if self.esp32.in_waiting > 0:
                line = self.esp32.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f"ESP32: {line}")
        except Exception as e:
            self.get_logger().error(f"Error UART: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()