from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    config_file = os.path.join(os.getcwd(), 'config', 'slam_toolbox_config.yaml')
    rviz_config = os.path.join(os.getcwd(), 'config', 'slam.rviz')
    
    return LaunchDescription([
        
        # Static TF: map -> odom (başlangıç için)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file],
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        # Static TF: base_link -> laser_link
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_laser',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
    output='screen'
),
    ])
