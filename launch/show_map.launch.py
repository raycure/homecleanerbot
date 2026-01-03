from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    
    map_file = os.path.join(os.getcwd(), 'maps', 'my_map.yaml')
    
    return LaunchDescription([
        
        # TF: base_link -> laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
        ),
        
        # TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file
            }]
        ),
        
        # Map Server Lifecycle (2 saniye bekle)
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                    output='screen'
                )
            ]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
