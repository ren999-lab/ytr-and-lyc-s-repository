# Copyright 2026 ytr
# Licensed under the Apache License, Version 2.0
"""Launch Gazebo Classic with the ABU ROBOCON 2027 arena world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition

class WorldType:
    RCMM = 'RCMM'

def get_world_config(world_type):
    world_configs = {
        WorldType.RCMM: {
            'x': '6.35',
            'y': '7.6',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': '27_rc_simulation/robocon2027_gazebo/worlds/robocon2027_arena.world'
        }
    }
    return world_configs.get(world_type, None)

def generate_launch_description():
    bringup_dir = get_package_share_directory('27_rc_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('27_rc_simulation'), 'urdf', 'simulation_waking_robot.xacro'
    )])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')
    robot_description = LaunchConfiguration('robot_description')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'True',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RCMM,
        description='Choose <RCMM> '
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )

    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )


    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )


    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', world_config['x'],
                        '-y', world_config['y'],
                        '-z', world_config['z'],
                        '-Y', world_config['yaw']
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': os.path.join(bringup_dir, 'world', world_config['world_path'])}.items(),
                )
            ]
        )


# def generate_launch_description():
#     pkg_share = get_package_share_directory('robocon2027_gazebo')
#     default_world = os.path.join(pkg_share, 'worlds', 'robocon2027_arena.world')

#     declare_world = DeclareLaunchArgument(
#         'world', default_value=default_world,
#         description='Full path to the SDF world to load')

#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('gazebo_ros'),
#                          'launch', 'gazebo.launch.py')
#         ),
#         launch_arguments={'world': LaunchConfiguration('world')}.items(),
#     )

#     return LaunchDescription([
#         declare_world,
#         gazebo_launch,
#     ])
