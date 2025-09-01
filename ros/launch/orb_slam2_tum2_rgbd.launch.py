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
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera',
        description='Camera topic prefix'
    )
    
    # Node configuration
    orb_slam2_rgbd_node = Node(
        package='orb_slam2_ros',
        executable='orb_slam2_ros_rgbd',
        name='orb_slam2_rgbd',
        output='screen',
        remappings=[
            ('camera/depth_registered/image_raw', 'camera/depth/image'),
            ('camera/rgb/image_raw', 'camera/rgb/image_color'),
        ],
        parameters=[{
            # Node parameters
            'publish_pointcloud': True,
            'publish_pose': True,
            'localize_only': False,
            'reset_map': False,
            
            # Static parameters
            'load_map': False,
            'map_file': 'map.bin',
            'settings_file': PathJoinSubstitution([pkg_share, 'orb_slam2/config/TUM2.yaml']),
            'voc_file': PathJoinSubstitution([pkg_share, 'orb_slam2/Vocabulary/ORBvoc.txt']),
            
            'pointcloud_frame_id': 'map',
            'camera_frame_id': 'camera_link',
            'min_num_kf_in_map': 5,
            
            # ORB SLAM2 parameters
            'ORBextractor.nFeatures': 2000,
            'ORBextractor.scaleFactor': 1.2,
            'ORBextractor.nLevels': 8,
            'ORBextractor.iniThFAST': 20,
            'ORBextractor.minThFAST': 7,
            
            # Camera parameters (TUM2 dataset)
            'camera_fx': 525.0,
            'camera_fy': 525.0,
            'camera_cx': 319.5,
            'camera_cy': 239.5,
            'camera_k1': 0.0,
            'camera_k2': 0.0,
            'camera_p1': 0.0,
            'camera_p2': 0.0,
            'camera_width': 640,
            'camera_height': 480,
            'depth_map_factor': 1.0,
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        orb_slam2_rgbd_node,
    ])
