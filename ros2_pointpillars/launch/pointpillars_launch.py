#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'checkpoint_path',
            default_value='/home/jiachenli/unitree_ros2/ros2_pointpillars/pretrained/epoch_160.pth',
            description='Path to the trained PointPillars model checkpoint'
        ),
        DeclareLaunchArgument(
            'use_cuda',
            default_value='true',
            description='Whether to use CUDA for inference'
        ),
        DeclareLaunchArgument(
            'point_cloud_range',
            default_value='[0, -39.68, -3, 69.12, 39.68, 1]',
            description='Point cloud filtering range [x_min, y_min, z_min, x_max, y_max, z_max]'
        ),
        DeclareLaunchArgument(
            'visualization_rate',
            default_value='10.0',
            description='Visualization update rate in Hz'
        ),
        
        # PointPillars node
        Node(
            package='ros2_pointpillars',
            executable='pointpillars_node',
            name='pointpillars_node',
            output='screen',
            parameters=[{
                'checkpoint_path': LaunchConfiguration('checkpoint_path'),
                'use_cuda': True,  # Fixed boolean value
                'point_cloud_range': [0, -39.68, -3, 69.12, 39.68, 1],  # Fixed list value
                'visualization_rate': 10.0,  # Fixed float value
            }],
        ),
    ]) 