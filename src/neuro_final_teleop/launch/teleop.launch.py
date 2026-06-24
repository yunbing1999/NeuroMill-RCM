from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    dry_run = LaunchConfiguration("dry_run")
    config_file = LaunchConfiguration("config")
    v7_ft_haptic_debug = LaunchConfiguration("v7_ft_haptic_debug")
    debug_topic_enable = LaunchConfiguration("debug_topic_enable")
    trigger_enable = LaunchConfiguration("trigger_enable")
    rumble_enable = LaunchConfiguration("rumble_enable")
    default_config = PathJoinSubstitution(
        [FindPackageShare("neuro_final_teleop"), "config", "neuro_final_default.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.1.243"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("v7_ft_haptic_debug", default_value="false"),
            DeclareLaunchArgument("debug_topic_enable", default_value="true"),
            DeclareLaunchArgument("trigger_enable", default_value="true"),
            DeclareLaunchArgument("rumble_enable", default_value="true"),
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
            ),
            Node(
                package="neuro_final_teleop",
                executable="neuro_final_teleop",
                name="neuro_final_teleop",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "robot_ip": robot_ip,
                        "dry_run": dry_run,
                        "v7_ft_haptic_debug": v7_ft_haptic_debug,
                        "v7_ft_haptic_debug_topic_enable": debug_topic_enable,
                        "v7_ft_haptic_trigger_enable": trigger_enable,
                        "v7_ft_haptic_rumble_enable": rumble_enable,
                    },
                ],
            ),
            Node(
                package="neuro_final_teleop",
                executable="session_gui",
                name="session_gui",
                output="screen",
            ),
        ]
    )
