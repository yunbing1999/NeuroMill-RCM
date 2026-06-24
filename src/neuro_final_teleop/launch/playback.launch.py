from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_time = {"use_sim_time": True}
    rviz_config = LaunchConfiguration("rviz_config")
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
                "hw_ns:=xarm ",
                "limited:=true ",
                "ros2_control_plugin:=false",
            ]
        )
    }

    zed_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("zed_description"), "launch", "zed_viz.launch.py"]
                )
            ]
        ),
        launch_arguments={"camera_model": "zed2i", "use_sim_time": "true"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            zed_description_launch,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description, sim_time],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="neuro_final_teleop",
                        executable="session_gui",
                        name="session_gui",
                        parameters=[sim_time],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[sim_time],
                output="screen",
            ),
        ]
    )
