#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry to TF publisher node started')
        self.get_logger().info('Subscribing to /odom topic')
        self.get_logger().info('Publishing odom -> base_link transform')
    
    def odom_callback(self, msg):
        """Convert odometry message to TF transform"""
        
        # Create transform message
        transform = TransformStamped()
        
        # Fill in the header
        transform.header.stamp = msg.header.stamp  # <-- Use odometry message timestamp here
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # Fill in the transform data from odometry
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        
        # Optional: Log position for debugging
        # self.get_logger().info(f'Publishing TF: x={transform.transform.translation.x:.3f}, y={transform.transform.translation.y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    
    node = OdomTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
