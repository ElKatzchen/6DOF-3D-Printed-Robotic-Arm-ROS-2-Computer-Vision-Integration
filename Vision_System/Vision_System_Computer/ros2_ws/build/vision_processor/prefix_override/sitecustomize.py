import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/katzchen/Documents/Taller/6DOF-3D-Printed-Robotic-Arm-ROS-2-Computer-Vision-Integration/Vision_System/Vision_System_Computer/ros2_ws/install/vision_processor'
