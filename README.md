# ros2_brickpi3
ROS2 packages to drive BrickPi3 (a Raspberry Pi to Lego EV3 hardware interface)


<img src=./brickpi3_charlie/images/accessorised_charlie.jpg width=300>

## YouTube Tutorial
<a href="https://www.youtube.com/watch?v=NZwhyERJVbY">
    <img src="https://img.youtube.com/vi/NZwhyERJVbY/0.jpg" height=320>
</a>

## Packages

|Package name|Description|
|------------|-----------|
|ev3_sensor_msgs|ROS2 custom message package to support the touch_sensor and color_sensor.|
|brickpi3_sensors|Python package to broadcast sensor messages from the BrickPi3 board. Currently supporting EV3 Touch, EV3 Ultrasonic, EV3 Infrared, EV3 Color, HiTechnic EV3/NXT Compass, EV3 Gyro, Battery Status.|
|brickpi3_motors|C++ package to provide a ROS2 Control hardware interface Plugin for the EV3 motors to the BrickPi3 board.|
|brickpi3_charlie|A demonstration robot. Includes an example launch file (and configuration files) to configure a ROS2 Control Differential Drive Controller to use the brickpi3_motors plugin.|

### Tested Hardware

Raspberry Pi 3 Model B+, Dexter Industries BrickPi3

### Tested Software

Ubuntu 22.04, ROS2 Jazzy, BrickPi3

## Installation

Note, the user account that you use for this install should be a member of the dialout group (in order to access /dev/spidev... to control the BrickPi3 hardware interface)

I suggest for following the installation steps to have the BrickPi3 running off an external power supply (rather than relying on battery power).

```
git clone -b microservices https://github.com/jfrancis71/ros2_brickpi3.git
```
```
docker build -t ros2_brickpi3 ./ros2_brickpi3/docker/brickpi3
```

### Troubleshooting

I suggest adding some temporary swap (I found 2GB perfectly sufficient). See discussion from Digital Ocean in the References section. Don't forget to remove the swap after a succesful installation. (A swap file on an SD card will reduce card life significantly)


## Verify install

```
docker run -it --rm --network=host --ipc=host --device=/dev/spidev0.1 ros2_brickpi3
```
```
source ./install/setup.bash
```

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

[Charlie](./brickpi3_charlie/README.md)


## References

YouTube video from Articulated Robots with overview of ROS2 control (accessed 26/01/2025):
[Solving the problem EVERY robot has (with ros2_control)](https://www.youtube.com/watch?v=4QKsDf1c4hc&t=1057s)


YouTube video from Articulated Robots with demo of how to write ROS2 Hardware Control Interface (accessed 26/01/2025):
[You can use ANY hardware with ros2_control](https://www.youtube.com/watch?v=J02jEKawE5U)


ROS2 Book:
Robot Programming with ROS2, Francisco Martin Rico, 2023.


Useful reference, summary of ROS2 commands:
[ROS2 Cheat Sheet](https://www.theroboticsspace.com/assets/article3/ros2_humble_cheat_sheet2.pdf)


Useful discussion on swap file on Ubuntu:
https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04
