from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('coal_face_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('period', default_value='40.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/coal_face.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'),
                              'period': LaunchConfiguration('period')}.items()),
        Node(package='coal_face_sim', executable='shearer_truth_tf.py',
             output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'config/coal_face_lidar.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
