# ORB-SLAM2 ROS2 Port

This is a ROS2 port of the ORB-SLAM2 ROS1 implementation. The port maintains the same functionality while adapting to ROS2's architecture and conventions.

## Key Changes from ROS1

### 1. Build System
- **ROS1**: `catkin` build system
- **ROS2**: `ament_cmake` build system

### 2. Dependencies
- **ROS1**: `roscpp`, `rospy`, `std_msgs`, `cv_bridge`, `image_transport`, `tf`, `tf2_ros`, `sensor_msgs`, `dynamic_reconfigure`
- **ROS2**: `rclcpp`, `rclpy`, `std_msgs`, `cv_bridge`, `image_transport`, `tf2`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`, `std_srvs`, `rcl_interfaces`

### 3. Node Architecture
- **ROS1**: Uses `ros::NodeHandle` and standalone node classes
- **ROS2**: Uses `rclcpp::Node` as base class with modern C++ patterns

### 4. Parameters
- **ROS1**: Uses `node_handle.param()` and `dynamic_reconfigure`
- **ROS2**: Uses `declare_parameter()` and `get_parameter()` with YAML configuration

### 5. Launch Files
- **ROS1**: XML format (`.launch`)
- **ROS2**: Python format (`.launch.py`)

### 6. Message Types
- **ROS1**: `sensor_msgs::Image`, `geometry_msgs::PoseStamped`, etc.
- **ROS2**: `sensor_msgs::msg::Image`, `geometry_msgs::msg::PoseStamped`, etc.

## Building

### Prerequisites
- ROS2 (tested with Humble and Iron)
- OpenCV 3.x or 4.x
- Eigen3
- Boost (for serialization)

### Build Instructions
```bash
# Clone the repository
git clone <repository-url>
cd orb_slam_2_ros

# Build with colcon
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

## Usage

### Monocular Camera
```bash
# Launch with default camera topic
ros2 launch orb_slam2_ros orb_slam2_mono.launch.py

# Launch with custom camera topic
ros2 launch orb_slam2_ros orb_slam2_mono.launch.py camera_topic:=/my_camera
```

### Stereo Camera
```bash
# Launch stereo node
ros2 launch orb_slam2_ros orb_slam2_stereo.launch.py
```

### RGB-D Camera
```bash
# Launch RGB-D node
ros2 launch orb_slam2_ros orb_slam2_tum2_rgbd.launch.py
```

## Docker (ROS 2 Humble)

Build the image from the repo root:

```bash
docker build -f orb_slam_2_ros/docker/humble/Dockerfile -t orb_slam2_ros:humble .
```

Run with host networking (adjust as needed):

```bash
docker run --rm -it \
  --net=host \
  --env=DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  orb_slam2_ros:humble bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# Example mono launch
ros2 launch orb_slam2_ros orb_slam2_mono.launch.py
```

## Topics

### Subscribed Topics
- **Monocular**: `/camera/image_raw` (sensor_msgs/msg/Image)
- **Stereo**: 
  - `/image_left/image_color_rect` (sensor_msgs/msg/Image)
  - `/image_right/image_color_rect` (sensor_msgs/msg/Image)
- **RGB-D**:
  - `/camera/rgb/image_raw` (sensor_msgs/msg/Image)
  - `/camera/depth_registered/image_raw` (sensor_msgs/msg/Image)

### Published Topics
- `/{node_name}/pose` (geometry_msgs/msg/PoseStamped) - Camera pose
- `/{node_name}/map_points` (sensor_msgs/msg/PointCloud2) - Map points
- `/{node_name}/debug_image` (sensor_msgs/msg/Image) - Debug visualization
- `/{node_name}/gba_running` (std_msgs/msg/Bool) - Global bundle adjustment status

### Services
- `/{node_name}/save_map` (orb_slam2_ros/srv/SaveMap) - Save current map

### Transforms
- Publishes transform from `map` frame to `camera_link` frame

## Parameters

### Node Parameters
- `publish_pointcloud` (bool, default: true) - Publish map points as point cloud
- `publish_pose` (bool, default: true) - Publish camera pose
- `publish_tf` (bool, default: true) - Publish transforms
- `pointcloud_frame_id` (string, default: "map") - Frame ID for point cloud
- `camera_frame_id` (string, default: "camera_link") - Camera frame ID
- `target_frame_id` (string, default: "base_link") - Target frame ID for transforms
- `load_map` (bool, default: false) - Load existing map on startup
- `map_file` (string, default: "map.bin") - Map file path

### ORB-SLAM2 Parameters
- `ORBextractor.nFeatures` (int, default: 2000) - Number of ORB features
- `ORBextractor.scaleFactor` (double, default: 1.2) - Scale factor between levels
- `ORBextractor.nLevels` (int, default: 8) - Number of pyramid levels
- `ORBextractor.iniThFAST` (int, default: 20) - Initial FAST threshold
- `ORBextractor.minThFAST` (int, default: 7) - Minimum FAST threshold

### Camera Parameters
- `camera_fx`, `camera_fy` (double) - Focal length
- `camera_cx`, `camera_cy` (double) - Principal point
- `camera_k1`, `camera_k2`, `camera_k3` (double) - Distortion coefficients
- `camera_p1`, `camera_p2` (double) - Tangential distortion coefficients

## Configuration

Parameters can be configured in several ways:

### 1. Launch File Parameters
```python
parameters=[{
    'publish_pointcloud': True,
    'ORBextractor.nFeatures': 2000,
    'camera_fx': 525.0,
    # ... other parameters
}]
```

### 2. YAML Configuration File
```yaml
/**:
  ros__parameters:
    publish_pointcloud: true
    ORBextractor.nFeatures: 2000
    camera_fx: 525.0
    # ... other parameters
```

### 3. Command Line
```bash
ros2 run orb_slam2_ros orb_slam2_ros_mono --ros-args -p publish_pointcloud:=false
```

## Differences from ROS1 Version

1. **No Dynamic Reconfigure**: ROS2 doesn't have dynamic_reconfigure. Use ROS2 parameters instead.
2. **Parameter System**: Uses ROS2's parameter system with YAML files.
3. **Launch Files**: Python-based launch files instead of XML.
4. **Message Types**: All message types use the `msg` namespace.
5. **Node Lifecycle**: Uses modern C++ patterns and smart pointers.

## Troubleshooting

### Common Issues

1. **Build Errors**: Ensure you have all dependencies installed:
   ```bash
   sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-tf2-ros
   ```

2. **Parameter Errors**: Check that all required parameters are declared in the node.

3. **Topic Remapping**: Use ROS2 remapping syntax in launch files or command line.

4. **Transform Issues**: Ensure TF2 is properly configured and frames are published.

### Debugging

Enable debug output:
```bash
ros2 run orb_slam2_ros orb_slam2_ros_mono --ros-args --log-level debug
```

## Contributing

When contributing to this ROS2 port:

1. Follow ROS2 coding conventions
2. Use modern C++ patterns (smart pointers, RAII)
3. Test with different ROS2 distributions
4. Update documentation for any new features

## License

This project maintains the same GPLv3 license as the original ORB-SLAM2 implementation.
