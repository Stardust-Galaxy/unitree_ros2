#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time

class CompleteSolutionTest(Node):
    def __init__(self):
        super().__init__('complete_solution_test')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to point cloud topic
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.point_cloud_callback,
            10
        )
        
        # Subscribe to detection markers
        self.detection_sub = self.create_subscription(
            MarkerArray,
            '/pointpillars_detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Complete Solution Test initialized')
        self.callback_count = 0
        self.detection_count = 0
        
        # Create a timer to publish TF transforms
        self.tf_timer = self.create_timer(0.1, self.publish_tf)
        
    def point_cloud_callback(self, msg):
        """Monitor point cloud and publish TF with Unitree B2 timestamps"""
        self.callback_count += 1
        
        if self.callback_count % 10 == 1:
            self.get_logger().info(f'📥 Point cloud #{self.callback_count}:')
            self.get_logger().info(f'   Frame: {msg.header.frame_id}')
            self.get_logger().info(f'   Unitree B2 timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            
            # Publish TF with Unitree B2 timestamp
            self.publish_tf_with_timestamp(msg.header.stamp)
    
    def detection_callback(self, msg):
        """Monitor detection markers"""
        self.detection_count += 1
        
        if len(msg.markers) > 0:
            marker = msg.markers[0]  # Check first marker
            self.get_logger().info(f'🎯 Detection #{self.detection_count}:')
            self.get_logger().info(f'   Marker timestamp: {marker.header.stamp.sec}.{marker.header.stamp.nanosec}')
            self.get_logger().info(f'   Frame: {marker.header.frame_id}')
            self.get_logger().info(f'   Number of markers: {len(msg.markers)}')
    
    def publish_tf_with_timestamp(self, timestamp):
        """Publish TF transforms with Unitree B2 timestamp"""
        try:
            # Base link to LiDAR transform
            lidar_transform = TransformStamped()
            lidar_transform.header.stamp = timestamp
            lidar_transform.header.frame_id = 'base_link'
            lidar_transform.child_frame_id = 'rslidar'
            
            lidar_transform.transform.translation.x = 0.0
            lidar_transform.transform.translation.y = 0.0
            lidar_transform.transform.translation.z = 0.5
            lidar_transform.transform.rotation.x = 0.0
            lidar_transform.transform.rotation.y = 0.0
            lidar_transform.transform.rotation.z = 0.0
            lidar_transform.transform.rotation.w = 1.0
            
            # Base link to map transform
            map_transform = TransformStamped()
            map_transform.header.stamp = timestamp
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
            
            if self.callback_count % 50 == 1:
                self.get_logger().info(f'📡 Published TF with Unitree B2 timestamp: map->base_link->rslidar')
                
        except Exception as e:
            self.get_logger().warn(f'Failed to publish TF: {e}')
    
    def publish_tf(self):
        """Fallback TF publishing"""
        if self.callback_count == 0:
            current_time = self.get_clock().now()
            self.publish_tf_with_timestamp(current_time.to_msg())

def main(args=None):
    rclpy.init(args=args)
    
    node = CompleteSolutionTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 