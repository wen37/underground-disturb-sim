from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    """生成洞穴启动配置；无入参；返回 LaunchDescription。"""
    share=Path(get_package_share_directory('cave_eval'))
    # 优先采用配置，否则从包所在位置向上查找资源。
    resource_override = os.environ.get('CAVE_WORLD_PATH')
    candidates = [Path(resource_override).expanduser().resolve()] if resource_override else [
        parent / 'upstream/gazebo_cave_world' for parent in share.parents]
    upstream = next((path for path in candidates if (path / 'worlds/cave_world.world').is_file()), None)
    if upstream is None:
        raise RuntimeError('未找到洞穴资源，请设置 CAVE_WORLD_PATH 或恢复工作区 upstream 资源')
    gazebo=Path(get_package_share_directory('gazebo_ros'))
    original=LaunchConfiguration('original')
    instrumented=IfCondition(PythonExpression(["'", original, "' != 'true'"]))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('original',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',str(upstream/'worlds/models')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI',''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gazebo.launch.py')),
            launch_arguments={'world':PythonExpression(["'",str(upstream/'worlds/cave_world.world'),"' if '", original,"' == 'true' else '",str(share/'worlds/cave_instrumented.world'),"'"]),
                              'gui':LaunchConfiguration('gui'),
                              'params_file':str(share/'config/gazebo.yaml')}.items()),
        Node(package='cave_eval',executable='shearer_motion.py',condition=instrumented,
             parameters=[str(share/'config/motion.yaml'),{'use_sim_time':True}],output='screen'),
        Node(package='cave_eval',executable='shearer_truth_tf.py',condition=instrumented,
             parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='rviz2',executable='rviz2',name='cave_eval_rviz',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',str(share/'config/cave_eval.rviz')],parameters=[{'use_sim_time':True}]),
    ])
