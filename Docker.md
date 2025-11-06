## Experimental Docker Install

Note that Docker will allow any Docker user to effectively act as root. Not suitable for multi user systems.
Also note that in the below setup the container itself is running as root. You will want to take care with firewalls
and not run any applications which can execute arbitrary code from external sources.

### To install Docker BrickPi3

Tested on Raspberry Pi 3B+ on Ubuntu 22.04.5 LTS, Docker version 27.5.1

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
RUN apt install -y ros-jazzy-compressed-image-transport
RUN apt install -y ros-jazzy-rosbridge-server
RUN apt install -y ros-jazzy-web-video-server
WORKDIR /root/ros2_ws
```

```
cd docker
docker build -t ros2_brickpi3 .
```

The above build will take about 30 mins (on Raspberry Pi 3B+).

Start Docker container with:
```
docker run -it --rm --privileged --network=host --ipc=host -v ros2_ws:/root/ros2_ws -v $HOME/.gitconfig:/root/.gitconfig -v $HOME/.git-credentials:/root/.git-credentials ros2_brickpi3 /bin/bash
```

If on a limited memory device (such as Raspberry Pi 3B+) you might want to do below step with swap enabled (2GB seems to be enough). You should remove swap when completed to save SD card wear.

We can build ros2 brickpi3 (in the container) with:
```
mkdir src
git -C src clone https://github.com/jfrancis71/ros2_brickpi3.git
export BRICKPI3_ROOT_DIR=/root/BrickPi3
export MAKEFLAGS="-j 1"  # We have limited memory on Raspberry Pi 3B+, best to build sequentially.
colcon build --symlink-install
```

Note it would have been convenient to do the above step within the Docker build file 'Dockerfile', but if we do this then the ros2_ws workspace will originate from the docker image and therefore not persist.

### To install Docker on Desktop

Tested on Ubuntu 22.04.5 LTS (Dell Precision Tower), Docker version 27.5.1

Create Dockerfile in folder docker:
```
FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y vim
RUN apt install -y python3-pip
RUN apt install -y ros-jazzy-teleop-twist-keyboard
RUN apt install -y ros-jazzy-teleop-twist-joy
RUN apt install -y ros-jazzy-image-tools
RUN apt install -y ros-jazzy-compressed-image-transport
RUN apt install -y ros-jazzy-rviz2
RUN apt install -y ros-jazzy-rosbridge-server
RUN apt install -y ros-jazzy-web-video-server
RUN apt install -y ros-jazzy-nav2-bringup
RUN apt install -y ros-jazzy-image-publisher
WORKDIR /root/ros2_ws
```

```
cd docker
docker build -t ros2 .
```

The above build will take about 30 mins (on Dell Precision Tower).

Start Docker container with:
```
docker run -it --rm --privileged --network=host --ipc=host -v ros2_ws:/root/ros2_ws -v $HOME/.gitconfig:/root/.gitconfig -v $HOME/.git-credentials:/root/.git-credentials -v="$XAUTHORITY:$XAUTHORITY" --env="XAUTHORITY=$XAUTHORITY"  --env="DISPLAY=$DISPLAY" ros2 /bin/bash
```

We also pass in our X Cookies and display device to allow us to run X11 programs (eg rviz2) and display them.

### General Notes

In both the above setups I allow container access to all devices, and use of host networking. We use a Docker volume (ros2_ws) to persist our workspace. I pass my git credentials into the container for convenience in pushing git commits to github. We remove (--rm) containers when we are done (to prevent proliferation of redundant containers). If ROS2 believes two nodes are running on the same machine it uses a shared memory optimization to communicate. If these nodes are running in seperate containers this will be blocked (by default) by Docker. Hence the --ipc flag to override and allow shared memory across containers.

# References

YouTube: https://www.youtube.com/watch?v=uf4zOigzTFo, Devices in Docker - Not so simple, Articulated Robotics, 06 Oct 2023
