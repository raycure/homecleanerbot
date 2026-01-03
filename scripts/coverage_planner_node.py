#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

class SimpleCoveragePlanner(Node):
    def __init__(self):
        super().__init__('simple_coverage_planner')
        
        # Parameters
        self.declare_parameter('row_spacing', 0.4)
        self.row_spacing = self.get_parameter('row_spacing').value
        
        # QoS profile for map (ÖNEMLI!)
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Map subscriber (QoS ile)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos  # QoS EKLEDIK!
        )
        
        # Services
        self.start_srv = self.create_service(
            Trigger,
            'start_cleaning',
            self.start_cleaning_callback
        )
        
        self.stop_srv = self.create_service(
            Trigger,
            'stop_cleaning',
            self.stop_cleaning_callback
        )
        
        # Navigation action client
        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )
        
        self.map_data = None
        self.cleaning_active = False
        self.goal_handle = None
        
        self.get_logger().info('='*50)
        self.get_logger().info('Simple Coverage Planner Ready!')
        self.get_logger().info('Waiting for map...')
        self.get_logger().info('='*50)
        
        # Timer to check map
        self.create_timer(2.0, self.check_map_status)
    
    def check_map_status(self):
        """Periodically check if map has been received"""
        if self.map_data is None:
            self.get_logger().warn('Still waiting for map on /map topic...', throttle_duration_sec=5.0)
        else:
            self.get_logger().info(
                f'Map ready: {self.map_data.info.width}x{self.map_data.info.height} '
                f'@ {self.map_data.info.resolution}m/cell',
                throttle_duration_sec=10.0
            )
    
    def map_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info(f'Map received! Size: {msg.info.width}x{msg.info.height}')
        self.map_data = msg
    
    def generate_coverage_path(self):
        """Generate boustrophedon (zigzag) coverage path"""
        if self.map_data is None:
            self.get_logger().error('No map available!')
            return []
        
        resolution = self.map_data.info.resolution
        width = self.map_data.info.width
        height = self.map_data.info.height
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        self.get_logger().info(f'Generating coverage path for map: {width}x{height}')
        
        # Find free space bounds
        free_cells = []
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                # 0 = free, 100 = occupied, -1 = unknown
                if self.map_data.data[idx] == 0:
                    free_cells.append((x, y))
        
        if not free_cells:
            self.get_logger().error('No free space found in map!')
            return []
        
        self.get_logger().info(f'Found {len(free_cells)} free cells')
        
        # Get bounds
        xs = [c[0] for c in free_cells]
        ys = [c[1] for c in free_cells]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        self.get_logger().info(f'Coverage bounds: X[{min_x}-{max_x}] Y[{min_y}-{max_y}]')
        
        # Generate waypoints (boustrophedon pattern)
        waypoints = []
        row_spacing_cells = max(1, int(self.row_spacing / resolution))
        
        self.get_logger().info(f'Row spacing: {row_spacing_cells} cells ({self.row_spacing}m)')
        
        going_right = True
        for y in range(min_y, max_y, row_spacing_cells):
            if going_right:
                x_range = range(min_x, max_x, row_spacing_cells)
            else:
                x_range = range(max_x, min_x, -row_spacing_cells)
            
            for x in x_range:
                # Convert to world coordinates
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                waypoints.append((world_x, world_y))
            
            going_right = not going_right
        
        self.get_logger().info(f'Generated {len(waypoints)} coverage waypoints')
        return waypoints
    
    def start_cleaning_callback(self, request, response):
        self.get_logger().info('='*50)
        self.get_logger().info('START CLEANING requested')
        self.get_logger().info('='*50)
        
        if self.cleaning_active:
            response.success = False
            response.message = 'Cleaning already in progress'
            self.get_logger().warn('Cleaning already in progress!')
            return response
        
        # Check if map is available
        if self.map_data is None:
            self.get_logger().error('Cannot start cleaning: No map available!')
            response.success = False
            response.message = 'No map available. Please ensure map_server is running.'
            return response
        
        # Generate path
        self.get_logger().info('Generating coverage path...')
        waypoints = self.generate_coverage_path()
        
        if not waypoints:
            response.success = False
            response.message = 'Failed to generate coverage path - no free space found'
            self.get_logger().error('Failed to generate coverage path')
            return response
        
        # Convert to poses
        poses = []
        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        
        # Send to navigator
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        
        self.get_logger().info('Waiting for navigation action server...')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            response.success = False
            response.message = 'Navigation action server not available'
            return response
        
        self.get_logger().info(f'Sending {len(poses)} waypoints to navigator...')
        self.cleaning_active = True
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        response.success = True
        response.message = f'Coverage cleaning started with {len(poses)} waypoints'
        self.get_logger().info(response.message)
        return response
    
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Coverage navigation REJECTED!')
            self.cleaning_active = False
            return
        
        self.get_logger().info('='*50)
        self.get_logger().info('Coverage navigation ACCEPTED')
        self.get_logger().info('Robot is now cleaning...')
        self.get_logger().info('='*50)
        
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        self.get_logger().info('='*50)
        self.get_logger().info('COVERAGE CLEANING COMPLETED!')
        self.get_logger().info('='*50)
        self.cleaning_active = False
        self.goal_handle = None
    
    def stop_cleaning_callback(self, request, response):
        self.get_logger().info('STOP CLEANING requested')
        
        if not self.cleaning_active:
            response.success = False
            response.message = 'No active cleaning to stop'
            self.get_logger().warn('No active cleaning to stop')
            return response
        
        if self.goal_handle:
            self.get_logger().info('Cancelling current navigation goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_callback)
        
        self.cleaning_active = False
        response.success = True
        response.message = 'Cleaning stopped'
        self.get_logger().info('Cleaning stopped')
        return response
    
    def cancel_callback(self, future):
        self.get_logger().info('Navigation goal cancelled')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
