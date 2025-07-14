#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import time

class UnitreeTFPublisher(Node):
    def __init__(self):
        super().__init__('unitree_tf_publisher')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to point cloud to get timestamps
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.point_cloud_callback,
            10
        )
        
        self.get_logger().info('Unitree TF Publisher initialized')
        self.get_logger().info('📡 Subscribed to /rslidar_points topic')
        self.callback_count = 0
        
        # Create a timer to publish static transforms
        self.timer = self.create_timer(0.1, self.publish_static_transforms)  # 10 Hz
        
        # Create a timer to check topic status
        self.status_timer = self.create_timer(5.0, self.check_topic_status)  # Check every 5 seconds
        
    def point_cloud_callback(self, msg):
        """Use point cloud timestamps for TF publishing"""
        self.callback_count += 1
        
        if self.callback_count % 10 == 1:
            self.get_logger().info(f'📥 Point cloud #{self.callback_count}: frame={msg.header.frame_id}')
            self.get_logger().info(f'   Unitree B2 timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        
        # Publish transforms using the point cloud timestamp from Unitree B2
        self.publish_transforms(msg.header.stamp)
        
        # Stop the fallback timer since we're receiving data
        if self.callback_count == 1:
            self.timer.cancel()
            self.get_logger().info('✅ Point cloud data received, using Unitree B2 timestamps')
    
    def publish_static_transforms(self):
        """Publish static transforms when no point cloud is available"""
        # Use current time only as fallback when no point cloud data is available
        current_time = self.get_clock().now()
        self.publish_transforms(current_time.to_msg())
        
        if self.callback_count == 0:
            self.get_logger().info('⚠️  Using fallback timestamp (no point cloud data yet)')
        else:
            self.get_logger().info('⚠️  Using fallback timestamp (point cloud data stopped)')
    
    def publish_transforms(self, timestamp):
        """Publish the necessary TF transforms for Unitree B2"""
        try:
            # 1. Base link to LiDAR transform
            lidar_transform = TransformStamped()
            lidar_transform.header.stamp = timestamp  # Use Unitree B2 timestamp
            lidar_transform.header.frame_id = 'base_link'
            lidar_transform.child_frame_id = 'rslidar'
            
            # Typical LiDAR mounting position on Unitree B2
            # Adjust these values based on your actual robot configuration
            lidar_transform.transform.translation.x = 0.0  # Forward
            lidar_transform.transform.translation.y = 0.0  # Left/Right
            lidar_transform.transform.translation.z = 0.5  # Height above base
            lidar_transform.transform.rotation.x = 0.0
            lidar_transform.transform.rotation.y = 0.0
            lidar_transform.transform.rotation.z = 0.0
            lidar_transform.transform.rotation.w = 1.0
            
            # 2. Base link to map transform (identity for now)
            map_transform = TransformStamped()
            map_transform.header.stamp = timestamp  # Use Unitree B2 timestamp
            map_transform.header.frame_id = 'map'
            map_transform.child_frame_id = 'base_link'
            
            map_transform.transform.translation.x = 0.0
            map_transform.transform.translation.y = 0.0
            map_transform.transform.translation.z = 0.0
            map_transform.transform.rotation.x = 0.0
            map_transform.transform.rotation.y = 0.0
            map_transform.transform.rotation.z = 0.0
            map_transform.transform.rotation.w = 1.0
            
            # Publish transforms
            self.tf_broadcaster.sendTransform(lidar_transform)
            self.tf_broadcaster.sendTransform(map_transform)
            
            if self.callback_count % 50 == 1:  # Log occasionally
                self.get_logger().info(f'📡 Published TF transforms with Unitree B2 timestamp: map->base_link->rslidar')
                
        except Exception as e:
            self.get_logger().warn(f'Failed to publish TF transforms: {e}')

    def check_topic_status(self):
        """Check if point cloud topic is available"""
        try:
            # Get topic info
            topic_info = self.get_topic_names_and_types()
            point_cloud_topics = [topic for topic, types in topic_info if 'rslidar_points' in topic]
            
            if point_cloud_topics:
                self.get_logger().info(f'✅ Point cloud topic found: {point_cloud_topics}')
            else:
                self.get_logger().warn(f'⚠️  Point cloud topic /rslidar_points not found')
                self.get_logger().info(f'Available topics: {[topic for topic, types in topic_info]}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to check topic status: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = UnitreeTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 