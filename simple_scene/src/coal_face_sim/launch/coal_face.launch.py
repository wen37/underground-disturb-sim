"""Launch the self-contained Classic scene and kinematic motion controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('coal_face_sim')
    gazebo = get_package_share_directory('gazebo_ros')
    model_path = os.path.join(share, 'models')
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('period', default_value='40.0'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': os.path.join(share, 'worlds', 'coal_face_motion.world'),
                'gui': LaunchConfiguration('gui'),
                'params_file': os.path.join(share, 'config', 'gazebo.yaml'),
            }.items()),
        Node(package='coal_face_sim', executable='shearer_motion.py',
             output='screen', parameters=[os.path.join(share, 'config', 'motion.yaml'),
                 {'use_sim_time': True, 'period': LaunchConfiguration('period')}]),
    ])
