#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')
        
        # PointPillars expected range (KITTI format)
        self.expected_range = [0, -39.68, -3, 69.12, 39.68, 1]
        
        # Subscribe to original point cloud
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.point_cloud_callback,
            10
        )
        
        # Publisher for transformed point cloud
        self.transformed_pub = self.create_publisher(
            PointCloud2,
            '/rslidar_points_transformed',
            10
        )
        
        self.get_logger().info('Coordinate Transformer initialized')
        self.get_logger().info('Transforming Unitree B2 LiDAR coordinates to PointPillars format')
        
    def point_cloud_callback(self, msg):
        """Transform point cloud coordinates"""
        # Parse original point cloud
        points = self.pointcloud2_to_numpy(msg)
        
        if len(points) == 0:
            return
            
        # Transform coordinates
        transformed_points = self.transform_coordinates(points)
        
        # Create new PointCloud2 message
        transformed_msg = self.numpy_to_pointcloud2(transformed_points, msg.header)
        
        # Publish transformed point cloud
        self.transformed_pub.publish(transformed_msg)
        
        # Log transformation statistics
        if len(transformed_points) > 0:
            x_range = (transformed_points[:, 0].min(), transformed_points[:, 0].max())
            y_range = (transformed_points[:, 1].min(), transformed_points[:, 1].max())
            z_range = (transformed_points[:, 2].min(), transformed_points[:, 2].max())
            
            self.get_logger().info(f'Transformed {len(transformed_points)} points')
            self.get_logger().info(f'New ranges: X[{x_range[0]:.2f}, {x_range[1]:.2f}], Y[{y_range[0]:.2f}, {y_range[1]:.2f}], Z[{z_range[0]:.2f}, {z_range[1]:.2f}]')
    
    def transform_coordinates(self, points):
        """Transform Unitree B2 coordinates to PointPillars format"""
        if len(points) == 0:
            return points
            
        # Get current ranges
        x_coords = points[:, 0]
        y_coords = points[:, 1]
        z_coords = points[:, 2]
        intensities = points[:, 3]
        
        current_x_range = (x_coords.min(), x_coords.max())
        current_y_range = (y_coords.min(), y_coords.max())
        current_z_range = (z_coords.min(), z_coords.max())
        
        self.get_logger().info(f'Original ranges: X[{current_x_range[0]:.2f}, {current_x_range[1]:.2f}], Y[{current_y_range[0]:.2f}, {current_y_range[1]:.2f}], Z[{current_z_range[0]:.2f}, {current_z_range[1]:.2f}]')
        
        # Transform X coordinates: shift to make minimum 0
        # Current: [-68.30, 62.38] -> Target: [0, 69.12]
        x_shift = -current_x_range[0]  # Shift to make minimum 0
        x_scale = self.expected_range[3] / (current_x_range[1] - current_x_range[0])  # Scale to fit range
        
        # Transform Y coordinates: center around 0
        # Current: [-29.12, 50.83] -> Target: [-39.68, 39.68]
        y_center = (current_y_range[0] + current_y_range[1]) / 2
        y_scale = (self.expected_range[4] - self.expected_range[1]) / (current_y_range[1] - current_y_range[0])
        
        # Transform Z coordinates: clip to expected range
        # Current: [-4.51, 12.79] -> Target: [-3, 1]
        z_clip_min = self.expected_range[2]
        z_clip_max = self.expected_range[5]
        
        # Apply transformations
        transformed_points = np.zeros_like(points)
        
        # X transformation
        transformed_points[:, 0] = (x_coords + x_shift) * x_scale
        
        # Y transformation
        transformed_points[:, 1] = (y_coords - y_center) * y_scale
        
        # Z transformation (clip to range)
        transformed_points[:, 2] = np.clip(z_coords, z_clip_min, z_clip_max)
        
        # Keep intensity unchanged
        transformed_points[:, 3] = intensities
        
        # Filter out points that are outside the expected range after transformation
        mask = (
            (transformed_points[:, 0] >= self.expected_range[0]) & 
            (transformed_points[:, 0] <= self.expected_range[3]) &
            (transformed_points[:, 1] >= self.expected_range[1]) & 
            (transformed_points[:, 1] <= self.expected_range[4]) &
            (transformed_points[:, 2] >= self.expected_range[2]) & 
            (transformed_points[:, 2] <= self.expected_range[5])
        )
        
        filtered_points = transformed_points[mask]
        
        self.get_logger().info(f'Filtered {len(filtered_points)}/{len(points)} points within expected range')
        
        return filtered_points
    
    def pointcloud2_to_numpy(self, pointcloud2_msg):
        """Convert ROS PointCloud2 message to numpy array"""
        points = []
        
        # Parse the point cloud fields
        field_names = [field.name for field in pointcloud2_msg.fields]
        field_offsets = [field.offset for field in pointcloud2_msg.fields]
        
        # Check if we have the required fields
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            self.get_logger().error(f'PointCloud2 message missing required fields. Available: {field_names}')
            return np.array([], dtype=np.float32)
        
        # Get field offsets
        x_offset = field_offsets[field_names.index('x')]
        y_offset = field_offsets[field_names.index('y')]
        z_offset = field_offsets[field_names.index('z')]
        intensity_offset = field_offsets[field_names.index('intensity')] if 'intensity' in field_names else -1
        
        # Determine the data format
        point_step = pointcloud2_msg.point_step
        data = pointcloud2_msg.data
        
        # Process all points
        for i in range(pointcloud2_msg.width * pointcloud2_msg.height):
            offset = i * point_step
            point_data = data[offset:offset + point_step]
            
            # Extract x, y, z coordinates using proper offsets
            x = struct.unpack('f', point_data[x_offset:x_offset + 4])[0]
            y = struct.unpack('f', point_data[y_offset:y_offset + 4])[0]
            z = struct.unpack('f', point_data[z_offset:z_offset + 4])[0]
            
            # Extract intensity if available, otherwise use 0
            intensity = 0.0
            if intensity_offset >= 0:
                intensity = struct.unpack('f', point_data[intensity_offset:intensity_offset + 4])[0]
            
            # Filter out invalid points (NaN or infinite values)
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                   np.isinf(x) or np.isinf(y) or np.isinf(z)):
                points.append([x, y, z, intensity])
        
        return np.array(points, dtype=np.float32)
    
    def numpy_to_pointcloud2(self, points, header):
        """Convert numpy array to ROS PointCloud2 message"""
        from sensor_msgs.msg import PointField
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'rslidar'  # Keep same frame ID
        
        # Set point cloud dimensions
        msg.height = 1
        msg.width = len(points)
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.point_step = 16  # 4 fields * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        
        # Convert points to bytes
        msg.data = points.astype(np.float32).tobytes()
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    transformer = CoordinateTransformer()
    
    try:
        rclpy.spin(transformer)
    except KeyboardInterrupt:
        pass
    finally:
        transformer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 