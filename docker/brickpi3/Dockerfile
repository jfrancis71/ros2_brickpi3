FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y vim
RUN apt install -y python3-pip
WORKDIR /root
RUN git clone https://github.com/DexterInd/BrickPi3.git
RUN pip install --break-system-packages BrickPi3/Software/Python
RUN apt install -y ros-jazzy-controller-manager
RUN apt install -y ros-jazzy-diff-drive-controller
RUN apt install -y ros-jazzy-bicycle-steering-controller
RUN apt install -y ros-jazzy-tricycle-steering-controller
RUN apt install -y ros-jazzy-mecanum-drive-controller
RUN apt install -y ros-jazzy-image-tools
RUN apt install -y ros-jazzy-compressed-image-transport
RUN apt install -y ros-jazzy-rosbridge-server
RUN apt install -y ros-jazzy-web-video-server

ENV BRICKPI3_ROOT_DIR=~/BrickPi3
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_brickpi3.git
# recommended to reduce memory usage.
ENV MAKEFLAGS="-j 1"
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select=brickpi3_motors"
