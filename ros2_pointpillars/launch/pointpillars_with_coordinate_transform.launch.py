#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Get the path to the package
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Declare launch arguments
    declare_checkpoint_path = DeclareLaunchArgument(
        'checkpoint_path',
        default_value='/home/jiachenli/unitree_ros2/ros2_pointpillars/pretrained/epoch_160.pth',
        description='Path to the PointPillars checkpoint file'
    )
    
    declare_use_cuda = DeclareLaunchArgument(
        'use_cuda',
        default_value='true',
        description='Whether to use CUDA for inference'
    )
    
    declare_visualization_enabled = DeclareLaunchArgument(
        'visualization_enabled',
        default_value='false',
        description='Whether to enable matplotlib visualization'
    )
    
    declare_visualization_rate = DeclareLaunchArgument(
        'visualization_rate',
        default_value='10.0',
        description='Visualization update rate in Hz'
    )
    
    declare_detection_frame_id = DeclareLaunchArgument(
        'detection_frame_id',
        default_value='rslidar',
        description='Frame ID for detection markers'
    )
    
    # Point cloud range parameters (KITTI format)
    declare_point_cloud_range_x_min = DeclareLaunchArgument(
        'point_cloud_range_x_min',
        default_value='0.0',
        description='Minimum X coordinate for point cloud range'
    )
    
    declare_point_cloud_range_y_min = DeclareLaunchArgument(
        'point_cloud_range_y_min',
        default_value='-39.68',
        description='Minimum Y coordinate for point cloud range'
    )
    
    declare_point_cloud_range_z_min = DeclareLaunchArgument(
        'point_cloud_range_z_min',
        default_value='-3.0',
        description='Minimum Z coordinate for point cloud range'
    )
    
    declare_point_cloud_range_x_max = DeclareLaunchArgument(
        'point_cloud_range_x_max',
        default_value='69.12',
        description='Maximum X coordinate for point cloud range'
    )
    
    declare_point_cloud_range_y_max = DeclareLaunchArgument(
        'point_cloud_range_y_max',
        default_value='39.68',
        description='Maximum Y coordinate for point cloud range'
    )
    
    declare_point_cloud_range_z_max = DeclareLaunchArgument(
        'point_cloud_range_z_max',
        default_value='1.0',
        description='Maximum Z coordinate for point cloud range'
    )
    
    # Coordinate Transformer Node
    coordinate_transformer_node = Node(
        package='ros2_pointpillars',
        executable='coordinate_transformer.py',
        name='coordinate_transformer',
        output='screen',
        parameters=[],
        remappings=[
            ('/rslidar_points', '/rslidar_points'),  # Input from Unitree B2
            ('/rslidar_points_transformed', '/rslidar_points_transformed')  # Output to PointPillars
        ]
    )
    
    # PointPillars Node
    pointpillars_node = Node(
        package='ros2_pointpillars',
        executable='ros2_pointpillars.py',
        name='pointpillars_node',
        output='screen',
        parameters=[
            {'checkpoint_path': LaunchConfiguration('checkpoint_path')},
            {'use_cuda': LaunchConfiguration('use_cuda')},
            {'visualization_enabled': LaunchConfiguration('visualization_enabled')},
            {'visualization_rate': LaunchConfiguration('visualization_rate')},
            {'detection_frame_id': LaunchConfiguration('detection_frame_id')},
            {'point_cloud_range_x_min': LaunchConfiguration('point_cloud_range_x_min')},
            {'point_cloud_range_y_min': LaunchConfiguration('point_cloud_range_y_min')},
            {'point_cloud_range_z_min': LaunchConfiguration('point_cloud_range_z_min')},
            {'point_cloud_range_x_max': LaunchConfiguration('point_cloud_range_x_max')},
            {'point_cloud_range_y_max': LaunchConfiguration('point_cloud_range_y_max')},
            {'point_cloud_range_z_max': LaunchConfiguration('point_cloud_range_z_max')}
        ],
        remappings=[
            ('/rslidar_points_transformed', '/rslidar_points_transformed')  # Input from coordinate transformer
        ]
    )
    
    # TF Publisher Node (for RViz visualization)
    tf_publisher_node = Node(
        package='ros2_pointpillars',
        executable='unitree_tf_publisher.py',
        name='unitree_tf_publisher',
        output='screen',
        parameters=[],
        remappings=[
            ('/rslidar_points', '/rslidar_points')  # Subscribe to original point cloud for timestamps
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_checkpoint_path,
        declare_use_cuda,
        declare_visualization_enabled,
        declare_visualization_rate,
        declare_detection_frame_id,
        declare_point_cloud_range_x_min,
        declare_point_cloud_range_y_min,
        declare_point_cloud_range_z_min,
        declare_point_cloud_range_x_max,
        declare_point_cloud_range_y_max,
        declare_point_cloud_range_z_max,
        
        # Nodes
        coordinate_transformer_node,
        pointpillars_node,
        tf_publisher_node
    ]) 