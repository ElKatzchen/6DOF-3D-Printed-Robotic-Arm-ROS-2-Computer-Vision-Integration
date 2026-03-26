import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/katzchen/Documents/Taller/6DOF-3D-Printed-Robotic-Arm-ROS-2-Computer-Vision-Integration/ROS2_System/ros2_ws/install/robotic_arm_control'
