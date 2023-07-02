import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',
                              default_value='true',
                              description='Open RViz.'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'launch/a-loam/lidar_ori.rviz']   
        ),
        Node(
            package='a-loam',
            namespace='aloam',
            executable='scan_registration_node',
            name='registration'
        ),
        Node(
            package='a-loam',
            namespace='aloam',
            executable='laser_odometry_node',
            name='odometry'
        ),
        Node(
            package='data-publisher',
            namespace='data',
            executable='lidar_data_node',
            name='lidar_data_ori'
        ),
    ])