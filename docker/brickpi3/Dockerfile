FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y vim
RUN apt install -y python3-pip
WORKDIR /root
RUN git clone https://github.com/DexterInd/BrickPi3.git
RUN pip install --break-system-packages BrickPi3/Software/Python
RUN apt install -y ros-jazzy-controller-manager
ENV BRICKPI3_ROOT_DIR=/root/BrickPi3
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_brickpi3.git
# recommended to reduce memory usage.
ENV MAKEFLAGS="-j 1"
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select brickpi3_motors brickpi3_sensors"
