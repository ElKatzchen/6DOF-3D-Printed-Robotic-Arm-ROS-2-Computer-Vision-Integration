import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

class SubNode(Node):
    def __init__(self):
        super().__init__('sub')
        # Configuración UART
        try:
            self.esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info("Conection UART with ESP32 stablished.")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir el puerto serie: {e}")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/joint_angles',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # OPCIÓN NUCLEAR: 2 Basura + 7 Reales
        # msg.data ya trae [Grip, S1, S2, S3, S4, S5, S6]
        full_vals = [0, 0] + list(msg.data)
        payload = "$" + "/".join([f"{int(v):03d}" for v in full_vals]) + "\n"
        
        try:
            self.esp32.reset_input_buffer()
            self.esp32.write(payload.encode())
            self.esp32.flush()
            self.get_logger().info(f"Enviando a ESP32: {payload.strip()}")
            
            # Leer ACK del ESP32
            time.sleep(0.05)
            if self.esp32.in_waiting > 0:
                line = self.esp32.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f"ESP32 responde: {line}")
        except Exception as e:
            self.get_logger().error(f"Error UART: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    rclpy.spin(node)
    rclpy.shutdown()