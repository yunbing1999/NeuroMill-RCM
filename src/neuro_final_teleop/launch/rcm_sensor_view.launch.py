"""Publish calibrated robot + approximate sensor geometry, without robot I/O."""
from pathlib import Path
import math
import tempfile
from neuro_final_teleop.drill_mesh import write_drill_mesh

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def build_nodes(context):
    height = float(LaunchConfiguration('sensor_height_m').perform(context))
    diameter = float(LaunchConfiguration('sensor_diameter_m').perform(context))
    if not (0 < height < 1 and 0 < diameter < 1):
        raise ValueError('Sensor dimensions must be positive metres below 1 m')
    shaft_length = float(LaunchConfiguration('shaft_length_m').perform(context))
    shaft_diameter = float(LaunchConfiguration('shaft_diameter_m').perform(context))
    if not (0 < shaft_length < 1 and 0 < shaft_diameter < 1):
        raise ValueError('Shaft dimensions must be positive metres below 1 m')
    yaw = float(LaunchConfiguration('mount_yaw_deg').perform(context))
    if not math.isfinite(yaw):
        raise ValueError('Mount yaw must be finite degrees')
    source = Path(get_package_share_directory('neuro_final_teleop')) / 'urdf/rcm_sensor_view.urdf.xacro'
    mesh_path = Path(tempfile.mkdtemp(prefix='neuromill_visual_')) / 'drill.stl'
    write_drill_mesh(mesh_path, yaw, height, shaft_length, shaft_diameter)
    description = xacro.process_file(str(source), mappings={
        'name': 'xarm7', 'robot_type': 'xarm', 'dof': '7',
        'add_gripper': 'false', 'add_vacuum_gripper': 'false',
        'kinematics_suffix': 'neuromill_check',
        'mount_yaw_deg': str(yaw),
        'drill_mesh_uri': mesh_path.as_uri(),
        'shaft_length_m': str(shaft_length), 'shaft_diameter_m': str(shaft_diameter),
        'sensor_height_m': str(height), 'sensor_diameter_m': str(diameter),
    }).toxml()
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 output='screen', parameters=[{'robot_description': description}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('shaft_length_m', default_value='0.10'),
        DeclareLaunchArgument('shaft_diameter_m', default_value='0.005'),
        DeclareLaunchArgument('sensor_height_m', default_value='0.056'),
        DeclareLaunchArgument('sensor_diameter_m', default_value='0.072'),
        DeclareLaunchArgument('mount_yaw_deg', default_value='-78.382',
                              description='Provisional visual mount yaw; verify against actual mounting'),
        OpaqueFunction(function=build_nodes),
    ])
