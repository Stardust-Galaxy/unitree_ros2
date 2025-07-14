#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import torch
import cv2
# import open3d as o3d  # Commented out for debugging
import threading
import time
import sys
import os
import queue
# Disable matplotlib by default to prevent memory issues
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# import tkinter as tk

# Add pointpillars to Python path
sys.path.insert(0, '/home/jiachenli/unitree_ros2/pointpillars')
sys.path.insert(0, '/home/jiachenli/unitree_ros2')
print(f"Python path: {sys.path}")

try:
    from pointpillars.model import PointPillars
    from pointpillars.utils import setup_seed, vis_pc, vis_img_3d
    from pointpillars.utils.process import bbox3d2corners_camera, points_camera2image
    from pointpillars.dataset import point_range_filter
    print("Successfully imported PointPillars modules")
except Exception as e:
    print(f"Failed to import PointPillars modules: {e}")
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
        self.declare_parameter('visualization_enabled', False)  # Disabled by default
        self.declare_parameter('detection_frame_id', 'rslidar')  # Frame ID for detections
        
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
        self.visualization_enabled = self.get_parameter('visualization_enabled').value
        self.detection_frame_id = self.get_parameter('detection_frame_id').value
        
        self.get_logger().info(f'Parameters loaded: checkpoint={self.checkpoint_path}, cuda={self.use_cuda}, viz={self.visualization_enabled}')
        self.get_logger().info(f'Detection frame ID: {self.detection_frame_id}')
        
        # Data storage
        self.latest_point_cloud = None
        self.latest_detections = None
        self.lock = threading.Lock()
        self._running = True
        self._callback_count = 0
        
        # Visualization thread (only if enabled)
        self.viz_queue = None
        self.viz_thread = None
        if self.visualization_enabled:
            self.start_visualization_thread()
        
        # Initialize components in order of dependency
        try:
            # Setup ROS2 publishers and subscribers FIRST (most critical)
            self.get_logger().info('Setting up ROS2 interface...')
            self.setup_ros_interface()
            self.get_logger().info('ROS2 interface setup complete')
            
            # Initialize model (optional for debugging)
            try:
                self.get_logger().info('Setting up model...')
                self.setup_model()
                self.get_logger().info('Model setup complete')
                
                # Test the model with dummy data
                self.get_logger().info('🧪 Testing model with dummy data...')
                dummy_points = np.array([[1.0, 0.0, 0.0, 100.0]], dtype='float32')
                test_result = self.process_point_cloud(dummy_points)
                if test_result is not None:
                    self.get_logger().info('✅ Model test successful!')
                else:
                    self.get_logger().warn('⚠️  Model test returned None')
                    
            except Exception as e:
                self.get_logger().warn(f'Model setup failed (continuing without model): {str(e)}')
                self.model = None
        
            self.get_logger().info('PointPillars ROS2 node initialized successfully')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize node: {str(e)}')
            # Ensure ROS2 interface is still available even if other components fail
            if not hasattr(self, 'point_cloud_sub'):
                self.setup_ros_interface()
            raise
    
    def setup_model(self):
        """Initialize the PointPillars model"""
        try:
            CLASSES = {
                'Pedestrian': 0, 
                'Cyclist': 1, 
                'Car': 2
            }
            
            if self.use_cuda and torch.cuda.is_available():
                self.model = PointPillars(nclasses=len(CLASSES), point_cloud_range=self.point_cloud_range).cuda()
                self.model.load_state_dict(torch.load(self.checkpoint_path))
                self.get_logger().info('Model loaded on CUDA')
            else:
                self.model = PointPillars(nclasses=len(CLASSES), point_cloud_range=self.point_cloud_range)
                self.model.load_state_dict(
                    torch.load(self.checkpoint_path, map_location=torch.device('cpu')))
                self.get_logger().info('Model loaded on CPU')
            
            self.model.eval()
            self.CLASSES = CLASSES
            self.LABEL2CLASSES = {v: k for k, v in CLASSES.items()}
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            raise
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers"""
        self.get_logger().info('Creating point cloud subscription...')
        
        # Subscribe to transformed point cloud topic (from coordinate transformer)
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points_transformed',
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
        
        # Reduce logging frequency to avoid spam
        if self._callback_count % 10 == 1:
            self.get_logger().info(f'📥 Point cloud callback #{self._callback_count} received')
            self.get_logger().info(f'   Frame: {msg.header.frame_id}, Stamp: {msg.header.stamp}')
            self.get_logger().info(f'   Size: {msg.width}x{msg.height}, Fields: {[f.name for f in msg.fields]}')
        
        if not self.is_ready():
            self.get_logger().warn('❌ Node not ready, skipping point cloud processing')
            return
            
        try:
            # Convert ROS PointCloud2 to numpy array
            points = self.pointcloud2_to_numpy(msg)
            
            # Validate point cloud data
            if points.size == 0:
                self.get_logger().warn('❌ Empty point cloud received')
                return
                
            # Show point cloud statistics (less frequently)
            if self._callback_count % 10 == 1:
                self.get_logger().info(f'📊 Point cloud statistics:')
                self.get_logger().info(f'   X range: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]')
                self.get_logger().info(f'   Y range: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]')
                self.get_logger().info(f'   Z range: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]')
                if points.shape[1] > 3:
                    self.get_logger().info(f'   Intensity range: [{points[:, 3].min():.2f}, {points[:, 3].max():.2f}]')
            
            # Filter points based on range
            data_dict = {'pts': points}
            data_dict = point_range_filter(data_dict, self.point_cloud_range)
            points = data_dict['pts']
            
            if len(points) == 0:
                self.get_logger().warn('❌ No points in range after filtering')
                return
            
            # Store latest point cloud
            with self.lock:
                self.latest_point_cloud = points
            
            # Process with PointPillars model (if available)
            if self.model is not None:
                detections = self.process_point_cloud(points)
                
                # Store latest detections
                with self.lock:
                    self.latest_detections = detections
                
                # Publish detection markers for RViz (only if we have detections)
                if detections is not None and len(detections['lidar_bboxes']) > 0:
                    self.publish_detection_markers(detections, msg.header)
                    
                    # Show detection results in console
                    num_detections = len(detections['lidar_bboxes'])
                    self.get_logger().info(f'🎯 DETECTIONS: {num_detections} objects found!')
                    for i, (bbox, label, score) in enumerate(zip(
                        detections['lidar_bboxes'], 
                        detections['labels'], 
                        detections['scores'])):
                        class_name = self.LABEL2CLASSES.get(label, f'Unknown_{label}')
                        self.get_logger().info(f'  📦 {class_name}: pos=({bbox[0]:.1f}, {bbox[1]:.1f}, {bbox[2]:.1f}), size=({bbox[3]:.1f}x{bbox[4]:.1f}x{bbox[5]:.1f}), score={score:.3f}')
                    
                    # Visualization: Show detections (if enabled and at the right frequency)
                    if self.visualization_enabled:
                        try:
                            # Update visualization with detections
                            self.update_visualization(points, detections)
                        except Exception as e:
                            self.get_logger().warn(f'❌ Detection visualization failed: {e}')
                else:
                    # Only log no detections occasionally to reduce spam
                    if self._callback_count % 50 == 1:
                        self.get_logger().info('🔍 No detections found in this frame')
            else:
                self.get_logger().warn('⚠️  Model not available, skipping inference')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error processing point cloud: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
    
    def pointcloud2_to_numpy(self, pointcloud2_msg):
        """Convert ROS PointCloud2 message to numpy array"""
        import struct
        
        # Get point cloud data
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
        
        # Debug: Log field information
        self.get_logger().info(f'Processing {pointcloud2_msg.width * pointcloud2_msg.height} points with {point_step} bytes per point')
        self.get_logger().info(f'Field offsets: x={x_offset}, y={y_offset}, z={z_offset}, intensity={intensity_offset}')
        self.get_logger().info(f'Available fields: {field_names}')
        
        # Sample first few points for debugging
        sample_points = min(5, pointcloud2_msg.width * pointcloud2_msg.height)
        for i in range(sample_points):
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
            
            self.get_logger().info(f'Sample point {i}: x={x:.3f}, y={y:.3f}, z={z:.3f}, intensity={intensity:.3f}')
            
            # Filter out invalid points (NaN or infinite values)
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                   np.isinf(x) or np.isinf(y) or np.isinf(z)):
                points.append([x, y, z, intensity])
        
        # Process remaining points
        for i in range(sample_points, pointcloud2_msg.width * pointcloud2_msg.height):
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
        
        result = np.array(points, dtype=np.float32)
        self.get_logger().info(f'Extracted {len(result)} valid points from {pointcloud2_msg.width * pointcloud2_msg.height} total points')
        
        # Additional validation
        if len(result) > 0:
            self.get_logger().info(f'Point cloud bounds: X[{result[:, 0].min():.2f}, {result[:, 0].max():.2f}], Y[{result[:, 1].min():.2f}, {result[:, 1].max():.2f}], Z[{result[:, 2].min():.2f}, {result[:, 2].max():.2f}]')
        
        return result
    
    def process_point_cloud(self, points):
        """Process point cloud through PointPillars model"""
        try:
            self.get_logger().info(f'🔧 Model input validation:')
            self.get_logger().info(f'   Input shape: {points.shape}')
            self.get_logger().info(f'   Data type: {points.dtype}')
            self.get_logger().info(f'   Memory usage: {points.nbytes / 1024 / 1024:.2f} MB')
            
            # Validate input data
            if points.size == 0:
                self.get_logger().error('❌ Empty point cloud provided to model')
                return None
                
            # Check data range
            x_range = (points[:, 0].min(), points[:, 0].max())
            y_range = (points[:, 1].min(), points[:, 1].max())
            z_range = (points[:, 2].min(), points[:, 2].max())
            self.get_logger().info(f'   X range: [{x_range[0]:.2f}, {x_range[1]:.2f}]')
            self.get_logger().info(f'   Y range: [{y_range[0]:.2f}, {y_range[1]:.2f}]')
            self.get_logger().info(f'   Z range: [{z_range[0]:.2f}, {z_range[1]:.2f}]')
            
            # Convert to torch tensor
            self.get_logger().info('🔄 Converting to PyTorch tensor...')
            pc_torch = torch.from_numpy(points).float()
            self.get_logger().info(f'   Tensor shape: {pc_torch.shape}')
            self.get_logger().info(f'   Tensor device: {pc_torch.device}')
            
            if self.use_cuda and torch.cuda.is_available():
                self.get_logger().info('🚀 Moving tensor to CUDA...')
                pc_torch = pc_torch.cuda()
                self.get_logger().info(f'   Tensor device after CUDA: {pc_torch.device}')
            else:
                self.get_logger().info('💻 Using CPU for inference')
            
            # Run inference
            self.get_logger().info('🧠 Running model inference...')
            with torch.no_grad():
                results = self.model(batched_pts=[pc_torch], mode='test')
                self.get_logger().info(f'   Model output type: {type(results)}')
                self.get_logger().info(f'   Model output length: {len(results) if isinstance(results, list) else "N/A"}')
                
                # Handle different output formats
                if isinstance(results, list):
                    if len(results) > 0:
                        self.get_logger().info(f'   First result type: {type(results[0])}')
                        if isinstance(results[0], dict):
                            result = results[0]  # List of dicts
                            self.get_logger().info('✅ Model returned list of dicts')
                        elif isinstance(results[0], (list, tuple)):
                            # Model returned list of lists/tuples (bboxes, labels, scores)
                            bboxes, labels, scores = results[0]
                            result = {
                                'lidar_bboxes': bboxes,
                                'labels': labels,
                                'scores': scores
                            }
                            self.get_logger().info('✅ Model returned list of lists/tuples')
                        else:
                            self.get_logger().error(f'❌ Unexpected list element type: {type(results[0])}')
                            return None
                    else:
                        self.get_logger().error('❌ Model returned empty list')
                        return None
                elif isinstance(results, dict):
                    result = results  # Direct dict
                    self.get_logger().info('✅ Model returned dict directly')
                else:
                    self.get_logger().error(f'❌ Unexpected model output type: {type(results)}')
                    return None
                
                self.get_logger().info(f'   Processed result keys: {list(result.keys()) if result else "None"}')
                
                # Debug: Check if we have valid detections
                if result and 'lidar_bboxes' in result:
                    num_bboxes = len(result['lidar_bboxes'])
                    self.get_logger().info(f'🎯 Model returned {num_bboxes} bounding boxes')
                    if num_bboxes > 0:
                        self.get_logger().info(f'   First bbox: {result["lidar_bboxes"][0]}')
                        self.get_logger().info(f'   Labels: {result["labels"]}')
                        self.get_logger().info(f'   Scores: {result["scores"]}')
                        
                        # Validate bbox format
                        bbox_shape = np.array(result["lidar_bboxes"]).shape
                        self.get_logger().info(f'   Bbox array shape: {bbox_shape}')
                        if len(bbox_shape) == 2 and bbox_shape[1] >= 6:
                            self.get_logger().info('✅ Bbox format looks correct (6+ dimensions)')
                        else:
                            self.get_logger().warn(f'⚠️  Unexpected bbox shape: {bbox_shape}')
                else:
                    self.get_logger().info('🔍 No detections in model output')
            
            # Filter detections based on range
            self.get_logger().info('🔍 Filtering detections based on range...')
            pcd_limit_range = np.array(self.point_cloud_range, dtype=np.float32)
            result = self.keep_bbox_from_lidar_range(result, pcd_limit_range)
            
            if result and 'lidar_bboxes' in result:
                final_count = len(result['lidar_bboxes'])
                self.get_logger().info(f'✅ Final result: {final_count} detections after filtering')
            else:
                self.get_logger().info('🔍 No detections after filtering')
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'❌ Error in model inference: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            return None
    
    def keep_bbox_from_lidar_range(self, result, pcd_limit_range):
        """Filter bounding boxes based on LiDAR range"""
        if result is None or len(result['lidar_bboxes']) == 0:
            return result
        
        bboxes = result['lidar_bboxes']
        labels = result['labels']
        scores = result['scores']
        
        # Filter based on center points
        centers = bboxes[:, :3]
        mask = (centers[:, 0] >= pcd_limit_range[0]) & (centers[:, 0] <= pcd_limit_range[3]) & \
               (centers[:, 1] >= pcd_limit_range[1]) & (centers[:, 1] <= pcd_limit_range[4]) & \
               (centers[:, 2] >= pcd_limit_range[2]) & (centers[:, 2] <= pcd_limit_range[5])
        
        result['lidar_bboxes'] = bboxes[mask]
        result['labels'] = labels[mask]
        result['scores'] = scores[mask]
        
        return result
    
    def publish_detection_markers(self, detections, header):
        """Publish detection results as RViz markers"""
        if detections is None or len(detections['lidar_bboxes']) == 0:
            return
        
        marker_array = MarkerArray()
        
        # Use the point cloud timestamp from Unitree B2 for proper synchronization
        # This ensures markers are synchronized with the sensor data
        point_cloud_timestamp = header.stamp
        
        for i, (bbox, label, score) in enumerate(zip(
            detections['lidar_bboxes'], 
            detections['labels'], 
            detections['scores'])):
            
            # Create bounding box marker
            marker = Marker()
            marker.header.stamp = point_cloud_timestamp  # Use Unitree B2 timestamp
            marker.header.frame_id = self.detection_frame_id  # Use rslidar frame directly
            marker.ns = "pointpillars_detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position and orientation
            marker.pose.position.x = float(bbox[0])
            marker.pose.position.y = float(bbox[1])
            marker.pose.position.z = float(bbox[2])
            
            # Set size
            marker.scale.x = float(bbox[3])  # length
            marker.scale.y = float(bbox[4])  # width
            marker.scale.z = float(bbox[5])  # height
            
            # Set color based on class
            if label == 0:  # Pedestrian
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
            elif label == 1:  # Cyclist
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
            elif label == 2:  # Car
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
            
            marker_array.markers.append(marker)
        
        self.detection_pub.publish(marker_array)
        self.get_logger().info(f'📤 Published {len(marker_array.markers)} detection markers with Unitree B2 timestamp')
    
    def start_visualization_thread(self):
        """Start the visualization thread"""
        if not self.visualization_enabled:
            self.get_logger().info('Visualization disabled, skipping thread creation')
            return
            
        try:
            import matplotlib.pyplot as plt
            import matplotlib.animation as animation
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            import tkinter as tk
            
            self.viz_queue = queue.Queue(maxsize=5)
            self.viz_thread = threading.Thread(target=self._visualization_worker, daemon=True)
            self.viz_thread.start()
            self.get_logger().info('Visualization thread started')
        except ImportError as e:
            self.get_logger().warn(f'Matplotlib not available, disabling visualization: {e}')
            self.visualization_enabled = False
    
    def _visualization_worker(self):
        """Worker thread for non-blocking visualization"""
        if not self.visualization_enabled:
            return
            
        fig = None
        try:
            import matplotlib.pyplot as plt
            plt.ion()  # Turn on interactive mode
            
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
            fig.suptitle('PointPillars ROS2 Visualization')
            
            while self._running:
                try:
                    # Get data from queue with timeout
                    data = self.viz_queue.get(timeout=1.0)
                    points, detections = data
                    
                    # Clear previous plots
                    ax1.clear()
                    ax2.clear()
            
                    # Plot point cloud (top-down view)
                    ax1.scatter(points[:, 0], points[:, 1], c=points[:, 3], s=0.1, alpha=0.6)
                    ax1.set_xlabel('X (m)')
                    ax1.set_ylabel('Y (m)')
                    ax1.set_title('Point Cloud (Top View)')
                    ax1.grid(True)
                    ax1.set_aspect('equal')
            
                    # Plot detections if available
                    if detections is not None and len(detections['lidar_bboxes']) > 0:
                        bboxes = detections['lidar_bboxes']
                        labels = detections['labels']
                        scores = detections['scores']
                        
                        colors = ['red', 'green', 'blue']
                        for i, (bbox, label, score) in enumerate(zip(bboxes, labels, scores)):
                            color = colors[label] if label < len(colors) else 'yellow'
                            # Draw bounding box as rectangle
                            x, y = bbox[0], bbox[1]
                            length, width = bbox[3], bbox[4]
                            rect = plt.Rectangle((x - length/2, y - width/2), length, width, 
                                               fill=False, color=color, linewidth=2)
                            ax2.add_patch(rect)
                            ax2.text(x, y, f'{self.LABEL2CLASSES.get(label, "Unknown")}\n{score:.2f}', 
                                    ha='center', va='center', color=color, fontweight='bold')
                        
                        ax2.set_xlabel('X (m)')
                        ax2.set_ylabel('Y (m)')
                        ax2.set_title(f'Detections ({len(bboxes)} objects)')
                        ax2.grid(True)
                        ax2.set_aspect('equal')
                    
                    plt.tight_layout()
                    plt.draw()
                    plt.pause(0.01)  # Small pause to allow GUI updates
                    
                except queue.Empty:
                    continue
                except Exception as e:
                    self.get_logger().warn(f'Visualization error: {e}')
            
            # Cleanup when thread ends
            if fig is not None:
                plt.close(fig)
                plt.close('all')  # Close all matplotlib figures
            
        except Exception as e:
            self.get_logger().error(f'Visualization thread failed: {e}')
            # Ensure cleanup even on error
            if fig is not None:
                plt.close(fig)
                plt.close('all')
    
    def update_visualization(self, points, detections=None):
        """Update visualization with new data (non-blocking)"""
        if not self.visualization_enabled or self.viz_queue is None:
            return
            
        try:
            # Try to put data in queue (non-blocking)
            self.viz_queue.put_nowait((points, detections))
        except queue.Full:
            # Queue is full, skip this update
            pass
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info('Destroying PointPillars node...')
        
        # Stop running flag
        self._running = False
        
        # Clear the visualization queue
        if hasattr(self, 'viz_queue'):
            while not self.viz_queue.empty():
                try:
                    self.viz_queue.get_nowait()
                except queue.Empty:
                    break
        
        # Wait for visualization thread to finish
        if self.viz_thread is not None and self.viz_thread.is_alive():
            self.get_logger().info('Waiting for visualization thread to finish...')
            self.viz_thread.join(timeout=3.0)
            if self.viz_thread.is_alive():
                self.get_logger().warn('Visualization thread did not finish gracefully')
        
        # Force cleanup matplotlib resources
        try:
            import matplotlib.pyplot as plt
            plt.close('all')  # Close all figures
            plt.ion()  # Reset interactive mode
        except Exception as e:
            self.get_logger().warn(f'Error cleaning up matplotlib: {e}')
        
        # Clear any remaining references
        if hasattr(self, 'viz_queue'):
            del self.viz_queue
        if hasattr(self, 'viz_thread'):
            del self.viz_thread
        
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