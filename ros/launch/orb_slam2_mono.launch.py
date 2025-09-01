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
    orb_slam2_mono_node = Node(
        package='orb_slam2_ros',
        executable='orb_slam2_ros_mono',
        name='orb_slam2_mono',
        output='screen',
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic') + '/image_raw'),
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
            
            # Camera parameters (default values)
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
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        orb_slam2_mono_node,
    ])
