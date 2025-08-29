#include "Node.h"

#include <iostream>

Node::Node (ORB_SLAM2::System::eSensor sensor, const std::string& node_name) 
  : rclcpp::Node(node_name), image_transport_(std::make_shared<image_transport::ImageTransport>(this)) {
  name_of_node_ = this->get_name();
  min_observations_per_point_ = 2;
  sensor_ = sensor;
}

Node::~Node () {
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}

void Node::Init () {
  DeclareParameters();
  
  // Get parameters
  publish_pointcloud_param_ = this->get_parameter("publish_pointcloud").as_bool();
  publish_pose_param_ = this->get_parameter("publish_pose").as_bool();
  publish_tf_param_ = this->get_parameter("publish_tf").as_bool();
  map_frame_id_param_ = this->get_parameter("pointcloud_frame_id").as_string();
  camera_frame_id_param_ = this->get_parameter("camera_frame_id").as_string();
  target_frame_id_param_ = this->get_parameter("target_frame_id").as_string();
  map_file_name_param_ = this->get_parameter("map_file").as_string();
  voc_file_name_param_ = this->get_parameter("voc_file").as_string();
  load_map_param_ = this->get_parameter("load_map").as_bool();

   // Create a parameters object to pass to the Tracking system
   ORB_SLAM2::ORBParameters parameters;
   LoadOrbParameters (parameters);

  orb_slam_ = new ORB_SLAM2::System (voc_file_name_param_, sensor_, parameters, map_file_name_param_, load_map_param_);

  service_server_ = this->create_service<orb_slam2_ros::srv::SaveMap>(
    name_of_node_ + "/save_map",
    std::bind(&Node::SaveMapSrv, this, std::placeholders::_1, std::placeholders::_2));

  // Initialization transformation listener
  tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  rendered_image_publisher_ = image_transport_->advertise (name_of_node_ + "/debug_image", 1);
  if (publish_pointcloud_param_) {
    map_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      name_of_node_ + "/map_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      name_of_node_ + "/pose", 1);
  }

  status_gba_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    name_of_node_ + "/gba_running", 1);
}

void Node::DeclareParameters() {
  // Declare all parameters with default values
  this->declare_parameter("publish_pointcloud", true);
  this->declare_parameter("publish_pose", true);
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("pointcloud_frame_id", "map");
  this->declare_parameter("camera_frame_id", "camera_link");
  this->declare_parameter("target_frame_id", "base_link");
  this->declare_parameter("map_file", "map.bin");
  this->declare_parameter("voc_file", "file_not_set");
  this->declare_parameter("load_map", false);
  this->declare_parameter("min_num_kf_in_map", 5);
  
  // ORB SLAM2 parameters
  this->declare_parameter("camera_fps", 30);
  this->declare_parameter("camera_rgb_encoding", true);
  this->declare_parameter("ORBextractor.nFeatures", 1200);
  this->declare_parameter("ORBextractor.scaleFactor", 1.2f);
  this->declare_parameter("ORBextractor.nLevels", 8);
  this->declare_parameter("ORBextractor.iniThFAST", 20);
  this->declare_parameter("ORBextractor.minThFAST", 7);
  this->declare_parameter("load_calibration_from_cam", false);
  this->declare_parameter("ThDepth", 35.0f);
  this->declare_parameter("depth_map_factor", 1.0f);
  
  // Camera calibration parameters
  this->declare_parameter("camera_baseline", 0.0);
  this->declare_parameter("camera_fx", 525.0);
  this->declare_parameter("camera_fy", 525.0);
  this->declare_parameter("camera_cx", 319.5);
  this->declare_parameter("camera_cy", 239.5);
  this->declare_parameter("camera_k1", 0.0);
  this->declare_parameter("camera_k2", 0.0);
  this->declare_parameter("camera_p1", 0.0);
  this->declare_parameter("camera_p2", 0.0);
  this->declare_parameter("camera_k3", 0.0);
}

void Node::Update () {
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    if (publish_tf_param_){
      PublishPositionAsTransform(position);
    }

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped(position);
    }
  }

  PublishRenderedImage (orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetAllMapPoints());
  }

  PublishGBAStatus (orb_slam_->isRunningGBA());
}

void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_->publish (cloud);
}

tf2::Transform Node::TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
  // Transform tf_in from frame_in to frame_target
  tf2::Transform tf_map2orig = tf_in;
  tf2::Transform tf_orig2target;
  tf2::Transform tf_map2target;

  tf2::Stamped<tf2::Transform> transformStamped_temp;
  try {
    // Get the transform from camera to target
    geometry_msgs::msg::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, tf2::TimePointZero);
    // Convert to tf2
    tf2::fromMsg(tf_msg, transformStamped_temp);
    tf_orig2target.setBasis(transformStamped_temp.getBasis());
    tf_orig2target.setOrigin(transformStamped_temp.getOrigin());

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    tf_orig2target.setIdentity();
  }

  // Transform from map to target
  tf_map2target = tf_map2orig * tf_orig2target;
  return tf_map2target;
}

void Node::PublishPositionAsTransform (cv::Mat position) {
  // Get transform from map to camera frame
  tf2::Transform tf_transform = TransformFromMat(position);

  // Make transform from camera frame to target frame
  tf2::Transform tf_map2target = TransformToTarget(tf_transform, camera_frame_id_param_, target_frame_id_param_);

  // Make message
  tf2::Stamped<tf2::Transform> tf_map2target_stamped;
  tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, current_frame_time_, map_frame_id_param_);
  geometry_msgs::msg::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
  msg.child_frame_id = target_frame_id_param_;
  // Broadcast tf
  tf_broadcaster_->sendTransform(msg);
}

void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  tf2::Transform tf_position = TransformFromMat(position);

  // Make transform from camera frame to target frame
  tf2::Transform tf_position_target = TransformToTarget(tf_position, camera_frame_id_param_, target_frame_id_param_);
  
  // Make message
  tf2::Stamped<tf2::Transform> tf_position_target_stamped;
  tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position_target, current_frame_time_, map_frame_id_param_);
  geometry_msgs::msg::PoseStamped pose_msg;
  tf2::toMsg(tf_position_target_stamped, pose_msg);
  pose_publisher_->publish(pose_msg);
}

void Node::PublishGBAStatus (bool gba_status) {
  std_msgs::msg::Bool gba_status_msg;
  gba_status_msg.data = gba_status;
  status_gba_publisher_->publish(gba_status_msg);
}

void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::msg::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::msg::Image::SharedPtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}

tf2::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

sensor_msgs::msg::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::msg::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}

bool Node::SaveMapSrv (const std::shared_ptr<orb_slam2_ros::srv::SaveMap::Request> request,
                       std::shared_ptr<orb_slam2_ros::srv::SaveMap::Response> response) {
  response->success = orb_slam_->SaveMap(request->name);

  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Map was saved as %s", request->name.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Map could not be saved.");
  }

  return response->success;
}

void Node::LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters) {
  //ORB SLAM configuration parameters
  parameters.maxFrames = this->get_parameter("camera_fps").as_int();
  parameters.RGB = this->get_parameter("camera_rgb_encoding").as_bool();
  parameters.nFeatures = this->get_parameter("ORBextractor.nFeatures").as_int();
  parameters.scaleFactor = this->get_parameter("ORBextractor.scaleFactor").as_double();
  parameters.nLevels = this->get_parameter("ORBextractor.nLevels").as_int();
  parameters.iniThFAST = this->get_parameter("ORBextractor.iniThFAST").as_int();
  parameters.minThFAST = this->get_parameter("ORBextractor.minThFAST").as_int();

  bool load_calibration_from_cam = this->get_parameter("load_calibration_from_cam").as_bool();

  if (sensor_== ORB_SLAM2::System::STEREO || sensor_==ORB_SLAM2::System::RGBD) {
    parameters.thDepth = this->get_parameter("ThDepth").as_double();
    parameters.depthMapFactor = this->get_parameter("depth_map_factor").as_double();
  }

  if (load_calibration_from_cam) {
    RCLCPP_INFO(this->get_logger(), "Listening for camera info on topic %s", camera_info_topic_.c_str());
    // Note: In ROS2, we would need to implement a proper camera info subscriber
    // For now, we'll use the parameters from the launch file
    RCLCPP_WARN(this->get_logger(), "Camera info loading not implemented in ROS2 version, using launch file params.");
  }

  bool got_cam_calibration = true;
  if (sensor_== ORB_SLAM2::System::STEREO || sensor_==ORB_SLAM2::System::RGBD) {
    parameters.baseline = this->get_parameter("camera_baseline").as_double();
  }

  parameters.fx = this->get_parameter("camera_fx").as_double();
  parameters.fy = this->get_parameter("camera_fy").as_double();
  parameters.cx = this->get_parameter("camera_cx").as_double();
  parameters.cy = this->get_parameter("camera_cy").as_double();
  parameters.k1 = this->get_parameter("camera_k1").as_double();
  parameters.k2 = this->get_parameter("camera_k2").as_double();
  parameters.p1 = this->get_parameter("camera_p1").as_double();
  parameters.p2 = this->get_parameter("camera_p2").as_double();
  parameters.k3 = this->get_parameter("camera_k3").as_double();

  if (!got_cam_calibration) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera calibration parameters from the launch file.");
    throw std::runtime_error("No cam calibration");
  }
}
