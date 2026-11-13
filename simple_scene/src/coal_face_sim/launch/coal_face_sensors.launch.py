from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('coal_face_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('period', default_value='40.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/coal_face_lidar.launch.py')),
            launch_arguments={k: LaunchConfiguration(k) for k in ['gui', 'rviz', 'period']}.items()),
        Node(package='coal_face_sim', executable='shearer_imu.py', output='screen',
             parameters=[{'use_sim_time': True, 'gravity_ms2': 9.81}]),
    ])
