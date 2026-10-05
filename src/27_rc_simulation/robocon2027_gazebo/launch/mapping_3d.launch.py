"""Mid360s 的3d建图启动文件"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,  SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction



def start_mapping(context):
    share = get_package_share_directory('robocon2027_gazebo')
    map_path = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('map_path').perform(context)
    ))
    if not map_path.lower().endswith('.pcd'):
        raise ValueError('map_path must end with .pcd')
    os.makedirs(os.path.dirname(map_path), exist_ok=True)
    return [Node(
        package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
        output='screen',
        parameters=[os.path.join(share, 'config', 'fastlio_rc_sim.yaml'),
                    {'use_sim_time': True, 'map_file_path': map_path}],
                    remappings=[('/Odometry', '/lio/odom')],
    )]
def generate_launch_description():
    share = get_package_share_directory('robocon2027_gazebo')

    get_package_share_directory('fast_lio')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rc_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'use_livox': 'true', 'rviz': 'false',
            'publish_odom_tf': 'false', 'gui': LaunchConfiguration('gui'),
            **{key: LaunchConfiguration(key) for key in ('x', 'y', 'z', 'yaw')},
        }.items(),
    )
    rviz = Node(
        package='rviz2',executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments= ['-d', os.path.join(share, 'rviz', 'mapping_3d.rviz')],
        parameters=[{'use_sim_time':True}],

    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='-5.15'),
        DeclareLaunchArgument('y', default_value='5.15'),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('yaw', default_value='-1.57079632679'),
        DeclareLaunchArgument('map_path', default_value=os.path.join(
            os.path.expanduser('~'), 'dev_ws', 'maps', 'rc_3d.pcd')),
        SetParameter(name='use_sim_time', value=True),
        GroupAction(scoped=True, actions=[simulation]), rviz,
        # Wait three simulation seconds for spawning/ground contact before IMU init.
        TimerAction(period=3.0, actions=[OpaqueFunction(function=start_mapping)]),
    ])
