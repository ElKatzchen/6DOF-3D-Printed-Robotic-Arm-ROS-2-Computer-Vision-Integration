docker compose down
docker compose up -d
docker compose exec ros2_env bash

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

echo $ROS_DOAMIN_ID
export ROS_DOAMIN_ID=0
echo $ROS_DOAMIN_ID
export ROS_IP=192.168.255.151
export NDDS_DISCOVERY_PEERS=udpv4://192.168.255.255

apt-get update && apt-get install -y python3-serial

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

v4l2-ctl --list-devices
export PYTHONPATH=$PYTHONPATH:/home/katzchen/miniconda3/envs/taller/lib/python3.10/site-packages