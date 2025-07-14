#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    checkpoint_path_arg = DeclareLaunchArgument(
        'checkpoint_path',
        default_value='/home/jiachenli/unitree_ros2/ros2_pointpillars/pretrained/epoch_160.pth',
        description='Path to the PointPillars model checkpoint'
    )
    
    use_cuda_arg = DeclareLaunchArgument(
        'use_cuda',
        default_value='true',
        description='Whether to use CUDA for model inference'
    )
    
    visualization_enabled_arg = DeclareLaunchArgument(
        'visualization_enabled',
        default_value='false',
        description='Whether to enable matplotlib visualization'
    )
    
    detection_frame_id_arg = DeclareLaunchArgument(
        'detection_frame_id',
        default_value='rslidar',
        description='Frame ID for detection markers'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms for detections'
    )
    
    # Point cloud range arguments
    pc_range_x_min_arg = DeclareLaunchArgument(
        'point_cloud_range_x_min',
        default_value='0.0',
        description='Minimum X coordinate for point cloud filtering'
    )
    
    pc_range_y_min_arg = DeclareLaunchArgument(
        'point_cloud_range_y_min',
        default_value='-39.68',
        description='Minimum Y coordinate for point cloud filtering'
    )
    
    pc_range_z_min_arg = DeclareLaunchArgument(
        'point_cloud_range_z_min',
        default_value='-3.0',
        description='Minimum Z coordinate for point cloud filtering'
    )
    
    pc_range_x_max_arg = DeclareLaunchArgument(
        'point_cloud_range_x_max',
        default_value='69.12',
        description='Maximum X coordinate for point cloud filtering'
    )
    
    pc_range_y_max_arg = DeclareLaunchArgument(
        'point_cloud_range_y_max',
        default_value='39.68',
        description='Maximum Y coordinate for point cloud filtering'
    )
    
    pc_range_z_max_arg = DeclareLaunchArgument(
        'point_cloud_range_z_max',
        default_value='1.0',
        description='Maximum Z coordinate for point cloud filtering'
    )
    
    # Create the TF Publisher node
    tf_publisher_node = Node(
        package='ros2_pointpillars',
        executable='unitree_tf_publisher',
        name='unitree_tf_publisher',
        output='screen',
    )
    
    # Create the PointPillars node
    pointpillars_node = Node(
        package='ros2_pointpillars',
        executable='pointpillars_node',
        name='pointpillars_node',
        output='screen',
        parameters=[{
            'checkpoint_path': LaunchConfiguration('checkpoint_path'),
            'use_cuda': LaunchConfiguration('use_cuda'),
            'visualization_enabled': LaunchConfiguration('visualization_enabled'),
            'detection_frame_id': LaunchConfiguration('detection_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'point_cloud_range_x_min': LaunchConfiguration('point_cloud_range_x_min'),
            'point_cloud_range_y_min': LaunchConfiguration('point_cloud_range_y_min'),
            'point_cloud_range_z_min': LaunchConfiguration('point_cloud_range_z_min'),
            'point_cloud_range_x_max': LaunchConfiguration('point_cloud_range_x_max'),
            'point_cloud_range_y_max': LaunchConfiguration('point_cloud_range_y_max'),
            'point_cloud_range_z_max': LaunchConfiguration('point_cloud_range_z_max'),
            'visualization_rate': 10.0,
        }],
        remappings=[
            ('/rslidar_points', '/rslidar_points'),  # Input point cloud topic
            ('/pointpillars_detections', '/pointpillars_detections'),  # Output detection markers
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        checkpoint_path_arg,
        use_cuda_arg,
        visualization_enabled_arg,
        detection_frame_id_arg,
        publish_tf_arg,
        pc_range_x_min_arg,
        pc_range_y_min_arg,
        pc_range_z_min_arg,
        pc_range_x_max_arg,
        pc_range_y_max_arg,
        pc_range_z_max_arg,
        
        # Nodes
        tf_publisher_node,
        pointpillars_node,
    ]) 