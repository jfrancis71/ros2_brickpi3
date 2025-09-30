## Experimental Docker Install

Note that Docker will allow any Docker user to effectively act as root. Not suitable for multi user systems.
Also note that in the below setup the container itself is running as root. You will want to take with firewalls
and not run any applications which can execute arbitrary code from external sources.

### To install Docker BrickPi3

Create Dockerfile in folder docker:
```
FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y vim
RUN apt install -y python3-pip
WORKDIR /root
RUN git clone https://github.com/DexterInd/BrickPi3.git
RUN pip install --break-system-packages BrickPi3/Software/Python
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_brickpi3.git
ENV BRICKPI3_ROOT_DIR=/root/BrickPi3
ENV MAKEFLAGS="-j 1"
RUN apt install -y ros-jazzy-controller-manager
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
RUN apt install -y ros-jazzy-diff-drive-controller
RUN apt install -y ros-jazzy-bicycle-steering-controller
RUN apt install -y ros-jazzy-tricycle-steering-controller
RUN apt install -y ros-jazzy-mecanum-drive-controller
RUN apt install -y ros-jazzy-image-tools
RUN apt install -y ros-jazzy-rosbridge-server
RUN apt install -y ros-jazzy-web-video-server
```

```
cd docker
docker build -t ros2_brickpi3 .
```

Start Docker container with:
```
docker run -it --rm --privileged -v ros2_ws:/home/ubuntu/ros2_ws -v $HOME/.gitconfig:/home/ubuntu/.gitconfig -v $HOME/.git-credentials:/home/ubuntu/.git-credentials --network=host --ipc=host ros2_brickpi3 /bin/bash
```
