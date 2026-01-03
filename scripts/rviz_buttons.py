#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class RVizButtons(Node):
    def __init__(self):
        super().__init__('rviz_buttons')
        
        # Service clients
        self.start_client = self.create_client(Trigger, 'start_cleaning')
        self.stop_client = self.create_client(Trigger, 'stop_cleaning')
        
        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, '/cleaning_buttons', 10)
        
        # Timer to publish markers
        self.create_timer(1.0, self.publish_buttons)
        
        self.get_logger().info('RViz Buttons: Click markers in RViz to control cleaning')
        self.get_logger().info('GREEN = Start Cleaning, RED = Stop Cleaning')
    
    def publish_buttons(self):
        # START button (Green)
        start_marker = Marker()
        start_marker.header.frame_id = 'map'
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = 'buttons'
        start_marker.id = 0
        start_marker.type = Marker.TEXT_VIEW_FACING
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = -2.0
        start_marker.pose.position.y = -2.0
        start_marker.pose.position.z = 1.0
        start_marker.scale.z = 0.5
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        start_marker.text = 'START CLEANING'
        
        self.marker_pub.publish(start_marker)
        
        # STOP button (Red)
        stop_marker = Marker()
        stop_marker.header.frame_id = 'map'
        stop_marker.header.stamp = self.get_clock().now().to_msg()
        stop_marker.ns = 'buttons'
        stop_marker.id = 1
        stop_marker.type = Marker.TEXT_VIEW_FACING
        stop_marker.action = Marker.ADD
        stop_marker.pose.position.x = 2.0
        stop_marker.pose.position.y = -2.0
        stop_marker.pose.position.z = 1.0
        stop_marker.scale.z = 0.5
        stop_marker.color.r = 1.0
        stop_marker.color.g = 0.0
        stop_marker.color.b = 0.0
        stop_marker.color.a = 1.0
        stop_marker.text = 'STOP CLEANING'
        
        self.marker_pub.publish(stop_marker)

def main(args=None):
    rclpy.init(args=args)
    node = RVizButtons()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
