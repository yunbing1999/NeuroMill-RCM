from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.1.243"),
            Node(
                package="neuro_final_teleop",
                executable="ft_bridge",
                name="ft_bridge",
                output="screen",
                parameters=[{"robot_ip": robot_ip, "publish_hz": 100.0}],
            ),
            Node(
                package="neuro_final_teleop",
                executable="joint_state_bridge",
                name="joint_state_bridge",
                output="screen",
                parameters=[{"robot_ip": robot_ip}],
            ),
        ]
    )
