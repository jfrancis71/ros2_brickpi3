# ros2_brickpi3
ROS2 packages to drive BrickPi3 (a Raspberry Pi to Lego EV3 hardware interface)

![Charlie](https://drive.google.com/uc?id=1HpBjWU5ElbmEphY0IHUyzJGjYCNfd94t&export=download)


## YouTube Tutorial
<a href="https://www.youtube.com/watch?v=NZwhyERJVbY">
    <img src="https://img.youtube.com/vi/NZwhyERJVbY/0.jpg" height=320>
</a>

## Packages

ev3_sensor_msgs: ROS2 custom message package to support the touch_sensor and color_sensor.

brickpi3_sensors: Python package to broadcast sensor messages from the BrickPi3 board. Currently supporting EV3 Touch, EV3 Ultrasonic, EV3 Infrared, EV3 Color, HiTechnic EV3/NXT Compass, EV3 Gyro, Battery Status.

brickpi3_motors: C++ package to provide a ROS2 Control hardware interface Plugin for the EV3 motors to the BrickPi3 board.

charlie: a demonstration robot. Includes an example launch file (and configuration files) to configure a ROS2 Control Differential Drive Controller to use the brickpi3_motors plugin.

### Tested Hardware

Raspberry Pi 3 Model B+, Dexter Industries BrickPi3

### Tested Software

Ubuntu 22.04, ROS2 Humble (RoboStack), BrickPi3

## Installation

Note, the user account that you use for this install should be a member of the dialout group (in order to access /dev/spidev... to control the BrickPi3 hardware interface)

I suggest for following the installation steps to have the BrickPi3 running off an external power supply (rather than relying on battery power).

The below instructions are for a RoboStack ROS2 humble install (for convenience). However, there is no dependency on RoboStack, any valid ROS2 Humble install should work.

Follow the [RoboStack](https://robostack.github.io/GettingStarted.html) installation instructions to install ROS2

(Ensure you have also followed the step Installation tools for local development in the above instructions)

```
mamba activate ros2  # (use the name here you decided to call this conda environment)
# scipy is used by the Gyro and Compass sensors. You can remove the scipy install if you do not intend to use
# these sensors.
mamba install scipy ros-humble-controller-manager
cd ~
git clone https://github.com/DexterInd/BrickPi3.git
pip install BrickPi3/Software/Python
mkdir -p ros2_ws/src
cd ros2_ws
git -C src clone https://github.com/jfrancis71/ros2_brickpi3.git
export BRICKPI3_ROOT_DIR=~/BrickPi3
colcon build --symlink-install
```
You may receive a warning on the colcon build step: "SetuptoolsDeprecationWarning: setup.py install is deprecated", this can be ignored.


## Activate Environment

```
mamba activate ros2 # (use the name here you decided to call this conda environment)
cd ~/ros2_ws
source ./install/setup.bash
```

## Verify install

This will test an EV3 infrared distance sensor connected to BrickPi3 sensor port 4.
```
ros2 run brickpi3_sensors infrared_distance_node --ros-args -p lego_port:=PORT_4
```
On another computer or shell window:
```
ros2 topic echo /infrared_distance
```
You should see messages being broadcasted giving the EV3 infrared distance sensor readings.

You can verify the motor installation by following the Charlie instructions.

## Charlie

Charlie is just a minimalist demonstration robot, primarily to demonstrate how to control the motors.
It is expected that you will wish to use your own repository to manage your own robots.

Charlie uses the ROS2 control Differential Drive controller to manage the hardware interface to the motors.
Therefore for this you will need to install:

```
mamba install ros-humble-diff-drive-controller ros-humble-rsl
source ./install/setup.bash  # source the ROS2 workspace again
```

To start the differential drive controller, run:
```
ros2 launch brickpi3_charlie brickpi3_motors_launch.py
```
Charlie's configuration files are setup to assume BrickPi3 ports A and D are connected to the left and right EV3 motors respectively.

On another computer or shell window:
```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
This should cause the motors to rotate (briefly).

For robots with other types of drive mechanisms, eg ackermann, you should install the appropriate ROS2 controller.

To control by keyboard:

```ros2 run teleop_twist_keyboard teleop_twist_keyboard```

To control by joystick:

Note the DiffDrive controller interprets twist messages in metric, so need to scale joystick (otherwise the speed commands will be far to fast for Charlie to be safe with). The below config file is based off the teleop_twist_joy/confix/xbox.config.yaml, but modified to scale linear.x
You may need to alter depending on your joystick model:

```ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=./src/ros2_brickpi3/brickpi3_charlie/config/xeox.config.yaml```

## Notes

RoboStack current build has the differential drive controller listening for Twist messages. The latest ROS2 codebase has this controller listening for TwistStamped messages, so you can expect this to change in future RoboStack releases. This imples some care should be taken when using different ROS2 builds that the correct Twist message is being processed.
