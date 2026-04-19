FROM ros2_brickpi3
RUN apt install -y ros-jazzy-diff-drive-controller
RUN git -C src clone --branch microservices https://github.com/jfrancis71/ros2_brickpi3
RUN colcon build --symlink-install --packages-select=brickpi3_charlie
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch brickpi3_charlie motors_launch.py"
