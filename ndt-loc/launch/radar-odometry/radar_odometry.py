from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',
                              default_value='true',
                              description='Open RViz.'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'launch/radar-odometry/radar.rviz']   
        ),
        Node(
            package='data-publisher',
            namespace='data',
            executable='radar_data_node',
            name='radar_data'
        ),
        Node(
            package='radar-odometry',
            namespace='radar',
            executable='radar_feature_node',
            name='radar_feature'
        ),
    ])