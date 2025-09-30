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
RUN apt install -y ros-jazzy-controller-manager
RUN apt install -y ros-jazzy-diff-drive-controller
RUN apt install -y ros-jazzy-bicycle-steering-controller
RUN apt install -y ros-jazzy-tricycle-steering-controller
RUN apt install -y ros-jazzy-mecanum-drive-controller
RUN apt install -y ros-jazzy-image-tools
RUN apt install -y ros-jazzy-rosbridge-server
RUN apt install -y ros-jazzy-web-video-server
WORKDIR /root/ros2_ws
```

```
cd docker
docker build -t ros2_brickpi3 .
```

Start Docker container with:
```
docker run -it --rm --privileged -v ros2_ws:/root/ros2_ws -v $HOME/.gitconfig:/root/.gitconfig -v $HOME/.git-credentials:/root/.git-credentials --network=host --ipc=host ros2_brickpi3 /bin/bash
```

We can build ros2 brickpi3 with:
```
mkdir src
git -C src clone https://github.com/jfrancis71/ros2_brickpi3.git
export BRICKPI3_ROOT_DIR=/root/BrickPi3
export MAKEFLAGS="-j 1"
colcon build --symlink-install
```

Note it would have been convenient to do the above step within Docker, but if we do this then the ros2_ws workspace will originate from the docker image and therefore not persist.

### To install Docker on Desktop

Create Dockerfile in folder docker:
```
FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y python3-pip
RUN apt install -y ros-jazzy-teleop-twist-keyboard
RUN apt install -y ros-jazzy-teleop-twist-joy
RUN apt install -y ros-jazzy-image-tools
RUN apt install -y ros-jazzy-rviz2
RUN apt install -y ros-jazzy-rosbridge-server
RUN apt install -y ros-jazzy-web-video-server
RUN apt install -y ros-jazzy-nav2-bringup
WORKDIR /root/ros2_ws
```

```
cd docker
docker build -t ros2 .
```

Start Docker container with:
```
docker run -it --rm --privileged -v ros2_ws:/root/ros2_ws -v $HOME/.gitconfig:/root/.gitconfig -v $HOME/.git-credentials:/root/.git-credentials -v="$XAUTHORITY:$XAUTHORITY" --network=host --ipc=host --env="XAUTHORITY=$XAUTHORITY"  --env="DISPLAY=$DISPLAY" ros2 /bin/bash
```
