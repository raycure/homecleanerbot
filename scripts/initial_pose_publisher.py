#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 5 saniye bekle (map_server ve AMCL başlasın)
        self.get_logger().info('Waiting 5 seconds before publishing initial pose...')
        time.sleep(5.0)
        
        self.publish_initial_pose()
        
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Robotun başlangıç pozisyonu (haritadaki konumuna göre ayarla)
        msg.pose.pose.position.x = 0.0  # X koordinatı
        msg.pose.pose.position.y = 0.0  # Y koordinatı
        msg.pose.pose.position.z = 0.0
        
        # Yön (quaternion)
        msg.pose.pose.orientation.w = 1.0  # 0 derece
        
        # Covariance (belirsizlik)
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        msg.pose.covariance[35] = 0.068  # yaw variance
        
        self.get_logger().info('Publishing initial pose: (0.0, 0.0)')
        self.publisher.publish(msg)
        
        # Birkaç kez yayınla (emin olmak için)
        time.sleep(0.5)
        self.publisher.publish(msg)
        time.sleep(0.5)
        self.publisher.publish(msg)
        
        self.get_logger().info('Initial pose published successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    time.sleep(2.0)  # 2 saniye daha bekle
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
