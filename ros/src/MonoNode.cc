#include "MonoNode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    if(argc > 1) {
        RCLCPP_WARN(rclcpp::get_logger("MonoNode"), "Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto node = std::make_shared<MonoNode>(ORB_SLAM2::System::MONOCULAR, "Mono");

    node->Init();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, const std::string& node_name) 
  : Node (sensor, node_name) {
  image_subscriber = image_transport_->subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";
}

MonoNode::~MonoNode () {
}

void MonoNode::ImageCallback (const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image, rclcpp::Time(msg->header.stamp).seconds());

  Update ();
}
