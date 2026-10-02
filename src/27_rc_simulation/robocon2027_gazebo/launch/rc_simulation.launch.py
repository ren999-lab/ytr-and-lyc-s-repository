#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition

# 当前只使用 RC 场地，保留你原来的分组启动结构。
class WorldType:
    RCMM = 'RCMM'


def get_world_config(world_type):
    """返回所选比赛场地对应的机器人出生位姿和 world 文件。"""
    world_configs = {
        WorldType.RCMM: {
            'x': '-5.15',
            'y': '5.15',
            'z': '0.2',
            'yaw': '-1.57079632679',
            'world_path': 'robocon2027_arena.world'
        },

    }
    return world_configs.get(world_type, None)

def generate_launch_description():
    """启动 RC 场地、原 RM 小车和 RViz；不依赖外部导航节点。"""
    # Get the launch directory
    # 这里是 package.xml 中的包名，不是外层文件夹名。
    # 你已把小车 urdf 复制到本包，因此小车和场地都从本包读取。
    bringup_dir = get_package_share_directory('robocon2027_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    default_robot_description = Command([
        FindExecutable(name='xacro'), ' "',
        os.path.join(bringup_dir, 'urdf', 'simulation_waking_robot.xacro'),
        '" use_livox:=', LaunchConfiguration('use_livox')
    ])

    # Create the launch configuration variables
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    use_rviz = LaunchConfiguration('rviz')
    # 强制按字符串传入 URDF，避免 ROS launch 把 XML 当成 YAML 参数解析。
    robot_description = ParameterValue(
        LaunchConfiguration('robot_description'), value_type=str)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RCMM,
        description='Choose <RCMM>',
        choices=[WorldType.RCMM]
    )

    declare_rviz_cmd = DeclareLaunchArgument('rviz', default_value='true')
    declare_gui_cmd = DeclareLaunchArgument('gui', default_value='true')
    declare_livox_cmd = DeclareLaunchArgument(
        'use_livox', default_value='false',
        description='Enable Mid360 after building ros2_livox_simulation and its driver dependency')
    world_config = get_world_config(WorldType.RCMM)
    declare_spawn_cmds = [
        DeclareLaunchArgument(key, default_value=world_config[key])
        for key in ('x', 'y', 'z', 'yaw')
    ]

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

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
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
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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
                        '-x', LaunchConfiguration('x'),
                        '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'),
                        '-Y', LaunchConfiguration('yaw'),
                        '-timeout', '120.0'
                    ],
                    output='screen',
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={
                        'world': os.path.join(bringup_dir, 'worlds', world_config['world_path'])
                    }.items(),
                )
            ]
        )

    bringup_RCMM_cmd_group = create_gazebo_launch_group(WorldType.RCMM)

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_livox_cmd)
    for declare_spawn_cmd in declare_spawn_cmds:
        ld.add_action(declare_spawn_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_RCMM_cmd_group)
    

    ld.add_action(start_rviz_cmd)

    return ld
