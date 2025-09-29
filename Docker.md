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
RUN apt install -y ros-jazzy-controller-manager
RUN apt install -y ros-jazzy-diff-drive-controller
RUN apt install -y ros-jazzy-tricycle-steering-controller
RUN apt install -y ros-jazzy-image-tools
RUN apt install -y ros-jazzy-rosbridge-server
RUN apt install -y ros-jazzy-web-video-server
WORKDIR /home/ubuntu
RUN git clone https://github.com/DexterInd/BrickPi3.git
RUN pip install --break-system-packages BrickPi3/Software/Python
WORKDIR /home/ubuntu/ros2_ws
```

```
cd docker
docker build -t ros2_brickpi3 .
```

Start Docker container with:
```
docker run -it --rm --privileged -v ros2_ws:/home/ubuntu/ros2_ws -v $HOME/.gitconfig:/home/ubuntu/.gitconfig -v $HOME
/.git-credentials:/home/ubuntu/.git-credentials --network=host --ipc=host ros2_brickpi3 /bin/bash
```
