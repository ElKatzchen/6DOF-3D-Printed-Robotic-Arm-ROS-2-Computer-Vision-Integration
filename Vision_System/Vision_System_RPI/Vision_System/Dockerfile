FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-serial \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout root

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]