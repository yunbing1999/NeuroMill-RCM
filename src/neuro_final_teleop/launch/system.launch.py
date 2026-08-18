from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    dry_run = LaunchConfiguration("dry_run")
    config_file = LaunchConfiguration("config")
    start_teleop = LaunchConfiguration("start_teleop")
    start_zed = LaunchConfiguration("start_zed")
    start_rviz = LaunchConfiguration("start_rviz")
    camera_model = LaunchConfiguration("camera_model")
    rviz_config = LaunchConfiguration("rviz_config")
    debug_topic_enable = LaunchConfiguration("debug_topic_enable")
    trigger_enable = LaunchConfiguration("trigger_enable")
    rumble_enable = LaunchConfiguration("rumble_enable")
    default_config = PathJoinSubstitution(
        [FindPackageShare("neuro_final_teleop"), "config", "neuro_final_default.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("neuro_final_teleop"), "rviz", "neuro_final.rviz"]
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("xarm_description"), "urdf", "xarm_device.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " ",
                "name:=xarm7 ",
                "robot_type:=xarm ",
                "dof:=7 ",
                "add_gripper:=false ",
                "add_vacuum_gripper:=false ",
                "hw_ns:=xarm ",
                "limited:=true ",
                "ros2_control_plugin:=false",
            ]
        )
    }

#    zed_camera_launch = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource(
#            [
#                PathJoinSubstitution(
#                    [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
#                )
#            ]
#        ),
#        launch_arguments={"camera_model": camera_model, "publish_tf": "true"}.items(),
#        condition=IfCondition(start_zed),
#    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.1.243"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            DeclareLaunchArgument("start_zed", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("camera_model", default_value="zed2i"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("debug_topic_enable", default_value="true"),
            DeclareLaunchArgument("trigger_enable", default_value="true"),
            DeclareLaunchArgument("rumble_enable", default_value="true"),
            zed_camera_launch,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
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
                        "v7_ft_haptic_debug_topic_enable": debug_topic_enable,
                        "v7_ft_haptic_trigger_enable": trigger_enable,
                        "v7_ft_haptic_rumble_enable": rumble_enable,
                    },
                ],
                condition=IfCondition(start_teleop),
            ),
            Node(
                package="neuro_final_teleop",
                executable="session_gui",
                name="session_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(start_rviz),
            ),
        ]
    )
