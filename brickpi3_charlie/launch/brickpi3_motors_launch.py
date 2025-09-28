from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        "cat ",
        PathJoinSubstitution(
            [FindPackageShare("brickpi3_charlie"), "config", "robot_hardware_description.urdf"])
        ])

    robot_controllers = PathJoinSubstitution(
            [FindPackageShare("brickpi3_charlie"), "config", "controller_description.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "differential_drive_controller",
            "--controller-ros-args",
            "-r /differential_drive_controller/cmd_vel:=/cmd_vel",
        ],
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner
    ]

    return LaunchDescription(nodes)

