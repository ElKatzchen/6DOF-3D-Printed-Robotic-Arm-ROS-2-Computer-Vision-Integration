docker compose down
docker compose up -d
docker compose exec ros2_env bash

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

echo $ROS_DOAMIN_ID
export ROS_DOAMIN_ID=0

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0