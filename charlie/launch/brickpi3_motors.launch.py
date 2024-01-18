from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        "cat ",
        PathJoinSubstitution(
            [FindPackageShare("charlie"), "config", "robot_hardware_description.urdf"])
        ])
    robot_controllers = PathJoinSubstitution(
            [FindPackageShare("charlie"), "config", "robot_description.yaml"])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description_content}, robot_controllers],
        remappings=[("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel")],
        output="both",
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )
    nodes = [
        control_node,
        robot_controller_spawner
    ]

    return LaunchDescription(nodes)

