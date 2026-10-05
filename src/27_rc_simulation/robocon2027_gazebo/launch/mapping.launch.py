"""Mid360 PointCloud2 -> LaserScan -> SLAM Toolbox, using Gazebo odometry."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robocon2027_gazebo')
    sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time = ParameterValue(sim_time, value_type=bool)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rc_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': sim_time, 'use_livox': 'true', 'rviz': 'false',
            'gui': LaunchConfiguration('gui'),
            'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'), 'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    converter = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', output='screen',
        parameters=[
            os.path.join(share, 'config', 'pointcloud_to_scan.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cloud_in', '/livox/lidar/pointcloud'), ('scan', '/scan')],
    )
    # Use the package's managed launch to configure and activate its lifecycle node.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': sim_time,
            'slam_params_file': os.path.join(share, 'config', 'slam_toolbox.yaml'),
        }.items(),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(share, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='-5.15'),
        DeclareLaunchArgument('y', default_value='5.15'),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('yaw', default_value='-1.57079632679'),
        simulation, converter, slam, rviz,
    ])