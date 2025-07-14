#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import threading
import time
import sys
import os

# Add pointpillars to Python path
sys.path.insert(0, '/home/jiachenli/unitree_ros2/pointpillars')
sys.path.insert(0, '/home/jiachenli/unitree_ros2')
print(f"Python path: {sys.path}")

# Create dummy functions for testing
def point_range_filter(data_dict, point_cloud_range):
    return data_dict


class PointPillarsROS2Node(Node):
    def __init__(self):
        super().__init__('pointpillars_ros2_node')
        
        self.get_logger().info('Starting PointPillars ROS2 node initialization...')
        
        # Initialize parameters
        self.declare_parameter('checkpoint_path', '/home/jiachenli/unitree_ros2/ros2_pointpillars/pretrained/epoch_160.pth')
        self.declare_parameter('use_cuda', True)
        # Use individual parameters for point cloud range to avoid type issues
        self.declare_parameter('point_cloud_range_x_min', 0.0)
        self.declare_parameter('point_cloud_range_y_min', -39.68)
        self.declare_parameter('point_cloud_range_z_min', -3.0)
        self.declare_parameter('point_cloud_range_x_max', 69.12)
        self.declare_parameter('point_cloud_range_y_max', 39.68)
        self.declare_parameter('point_cloud_range_z_max', 1.0)
        self.declare_parameter('visualization_rate', 10.0)  # Hz
        
        # Get parameters
        self.checkpoint_path = self.get_parameter('checkpoint_path').value
        self.use_cuda = self.get_parameter('use_cuda').value
        # Construct point cloud range from individual parameters
        self.point_cloud_range = [
            self.get_parameter('point_cloud_range_x_min').value,
            self.get_parameter('point_cloud_range_y_min').value,
            self.get_parameter('point_cloud_range_z_min').value,
            self.get_parameter('point_cloud_range_x_max').value,
            self.get_parameter('point_cloud_range_y_max').value,
            self.get_parameter('point_cloud_range_z_max').value
        ]
        self.visualization_rate = self.get_parameter('visualization_rate').value
        
        self.get_logger().info(f'Parameters loaded: checkpoint={self.checkpoint_path}, cuda={self.use_cuda}')
        
        # Data storage
        self.latest_point_cloud = None
        self.latest_detections = None
        self.lock = threading.Lock()
        self._running = True
        self._callback_count = 0
        
        # Initialize components in order of dependency
        try:
            # Setup ROS2 publishers and subscribers FIRST (most critical)
            self.get_logger().info('Setting up ROS2 interface...')
            self.setup_ros_interface()
            self.get_logger().info('ROS2 interface setup complete')
            
            # Skip model loading for debugging
            self.get_logger().info('Skipping model loading for debugging')
            self.model = None
            
            # Skip visualization for now
            self.get_logger().info('Skipping visualization setup for debugging')
            
            self.get_logger().info('PointPillars ROS2 node initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize node: {str(e)}')
            # Ensure ROS2 interface is still available even if other components fail
            if not hasattr(self, 'point_cloud_sub'):
                self.setup_ros_interface()
            raise
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers"""
        self.get_logger().info('Creating point cloud subscription...')
        
        # Subscribe to point cloud topic
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.point_cloud_callback,
            10
        )
        
        self.get_logger().info('Creating detection publisher...')
        
        # Publisher for detection markers (optional, for RViz visualization)
        self.detection_pub = self.create_publisher(
            MarkerArray,
            '/pointpillars_detections',
            10
        )
        
        self.get_logger().info('ROS2 interface setup complete')
    
    def is_ready(self):
        """Check if the node is ready to process data"""
        ready = (hasattr(self, 'point_cloud_sub') and 
                hasattr(self, 'detection_pub'))
        self.get_logger().info(f'Node ready check: {ready}')
        return ready
    
    def point_cloud_callback(self, msg):
        """Callback for processing incoming point cloud messages"""
        self._callback_count += 1
        self.get_logger().info(f'Point cloud callback received! Count: {self._callback_count}')
        self.get_logger().info(f'Message header: frame_id={msg.header.frame_id}, stamp={msg.header.stamp}')
        self.get_logger().info(f'Point cloud size: {msg.width}x{msg.height}, fields: {[f.name for f in msg.fields]}')
        
        if not self.is_ready():
            self.get_logger().warn('Node not ready, skipping point cloud processing')
            return
            
        try:
            # Convert ROS PointCloud2 to numpy array
            self.get_logger().info('Converting point cloud to numpy...')
            points = self.pointcloud2_to_numpy(msg)
            self.get_logger().info(f'Converted to numpy array with shape: {points.shape}')
            
            # Filter points based on range
            self.get_logger().info('Filtering points...')
            data_dict = {'pts': points}
            data_dict = point_range_filter(data_dict, self.point_cloud_range)
            points = data_dict['pts']
            self.get_logger().info(f'After filtering: {points.shape}')
            
            if len(points) == 0:
                self.get_logger().warn('No points in range after filtering')
                return
            
            # Store latest point cloud
            with self.lock:
                self.latest_point_cloud = points
            
            # Skip model processing for debugging
            self.get_logger().info('Model not available, skipping inference')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
    
    def pointcloud2_to_numpy(self, pointcloud2_msg):
        """Convert ROS PointCloud2 message to numpy array"""
        import struct
        
        # Get point cloud data
        points = []
        
        # Parse the point cloud fields
        field_names = [field.name for field in pointcloud2_msg.fields]
        
        # Check if we have the expected fields
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            self.get_logger().error(f'PointCloud2 message missing required fields. Available: {field_names}')
            return np.array([], dtype=np.float32)
        
        # Determine the data format
        point_step = pointcloud2_msg.point_step
        data = pointcloud2_msg.data
        
        self.get_logger().info(f'Processing {pointcloud2_msg.width * pointcloud2_msg.height} points')
        
        for i in range(pointcloud2_msg.width * pointcloud2_msg.height):
            offset = i * point_step
            point_data = data[offset:offset + point_step]
            
            # Extract x, y, z coordinates
            x = struct.unpack('f', point_data[0:4])[0]
            y = struct.unpack('f', point_data[4:8])[0]
            z = struct.unpack('f', point_data[8:12])[0]
            
            # Extract intensity if available, otherwise use 0
            intensity = 0.0
            if 'intensity' in field_names:
                intensity_offset = field_names.index('intensity') * 4
                if intensity_offset + 4 <= len(point_data):
                    intensity = struct.unpack('f', point_data[intensity_offset:intensity_offset + 4])[0]
            
            points.append([x, y, z, intensity])
        
        return np.array(points, dtype=np.float32)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info('Destroying PointPillars node...')
        
        # Stop running flag
        self._running = False
        
        self.get_logger().info(f'Total callbacks received: {self._callback_count}')
        self.get_logger().info('PointPillars node destroyed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = PointPillarsROS2Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 