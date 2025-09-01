#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('orb_slam2_ros')
    
    # Declare launch arguments
    left_topic_arg = DeclareLaunchArgument(
        'left_topic',
        default_value='/image_left/image_color_rect',
        description='Left camera image topic'
    )
    
    right_topic_arg = DeclareLaunchArgument(
        'right_topic',
        default_value='/image_right/image_color_rect',
        description='Right camera image topic'
    )
    
    # Node configuration
    orb_slam2_stereo_node = Node(
        package='orb_slam2_ros',
        executable='orb_slam2_ros_stereo',
        name='orb_slam2_stereo',
        output='screen',
        remappings=[
            ('image_left/image_color_rect', LaunchConfiguration('left_topic')),
            ('image_right/image_color_rect', LaunchConfiguration('right_topic')),
        ],
        parameters=[{
            # Node parameters
            'publish_pointcloud': True,
            'publish_pose': True,
            'publish_tf': True,
            
            # Static parameters
            'load_map': False,
            'map_file': 'map.bin',
            'voc_file': PathJoinSubstitution([pkg_share, 'orb_slam2/Vocabulary/ORBvoc.txt']),
            
            'pointcloud_frame_id': 'map',
            'camera_frame_id': 'camera_link',
            'target_frame_id': 'base_link',
            'min_num_kf_in_map': 5,
            
            # ORB SLAM2 parameters
            'ORBextractor.nFeatures': 2000,
            'ORBextractor.scaleFactor': 1.2,
            'ORBextractor.nLevels': 8,
            'ORBextractor.iniThFAST': 20,
            'ORBextractor.minThFAST': 7,
            
            # Camera parameters (default values - adjust for your camera)
            'camera_fx': 525.0,
            'camera_fy': 525.0,
            'camera_cx': 319.5,
            'camera_cy': 239.5,
            'camera_k1': 0.0,
            'camera_k2': 0.0,
            'camera_p1': 0.0,
            'camera_p2': 0.0,
            'camera_k3': 0.0,
            'camera_width': 640,
            'camera_height': 480,
            'camera_baseline': 0.12,  # Stereo baseline in meters
            'ThDepth': 35.0,  # Depth threshold for stereo
            'depth_map_factor': 1.0,
        }]
    )
    
    return LaunchDescription([
        left_topic_arg,
        right_topic_arg,
        orb_slam2_stereo_node,
    ])
