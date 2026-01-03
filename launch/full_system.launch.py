from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    
    nav2_params = os.path.join(os.getcwd(), 'config', 'nav2_params.yaml')
    map_file = os.path.join(os.getcwd(), 'maps', 'my_map.yaml')
    rviz_config = os.path.join(os.getcwd(), 'config', 'nav2.rviz')
    
    return LaunchDescription([
        
        # 1. Gazebo Simulator
        ExecuteProcess(
            cmd=['gz', 'sim', os.path.join(os.getcwd(), 'house.sdf')],
            output='screen',
            shell=False
        ),
        
        # Wait for Gazebo to start
        TimerAction(
            period=5.0,
            actions=[
                # 2. Gazebo-ROS Bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge',
                    arguments=[
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # 3. Scan Frame Fixer
        Node(
            package='simple_coverage_planner',  # veya direkt executable
            executable='scan_frame_fixer.py',
            name='scan_frame_fixer',
            output='screen'
        ),
        
        # 4. Odom to TF
        Node(
            package='simple_coverage_planner',  # veya direkt executable
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen'
        ),
        
        # 5. Static TF: base_link -> laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
            output='screen'
        ),
        # 5b. Static TF: map -> odom (YENİ EKLE!)
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_odom',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    output='screen',
    respawn=True,  # Ölürse yeniden başlar
    respawn_delay=2.0
),
        # 6. Map Server
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
        
        # 7. AMCL (Localization)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),
        
        # 8. Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),
        
        # 9. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),
        
        # 10. Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),
        
        # 11. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),
        
        # 12. Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params]
        ),
        
        # 13. Coverage Planner
        Node(
            package='simple_coverage_planner',
            executable='coverage_planner_node.py',
            name='coverage_planner',
            output='screen',
            parameters=[{
                'row_spacing': 0.4
            }]
        ),
        
        # 14. Lifecycle Manager (Map Server)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
        
        # 15. Lifecycle Manager (Navigation)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'amcl'
                ]
            }]
        ),
        # Initial Pose Publisher (AMCL için)
ExecuteProcess(
    cmd=['python3', os.path.join(os.getcwd(), 'scripts/initial_pose_publisher.py')],
    output='screen',
    shell=False
),

        # 16. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
