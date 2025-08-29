#include "StereoNode.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc > 1) {
        RCLCPP_WARN(rclcpp::get_logger("StereoNode"), "Arguments supplied via command line are neglected.");
    }

    // initialize
    auto node = std::make_shared<StereoNode>(ORB_SLAM2::System::STEREO, "Stereo");

    node->Init();

    rclcpp::spin(node);

    return 0;
}

StereoNode::StereoNode (const ORB_SLAM2::System::eSensor sensor, const std::string& node_name) 
  : Node (sensor, node_name) {
    left_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (this, "image_left/image_color_rect");
    right_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (this, "image_right/image_color_rect");
    camera_info_topic_ = "image_left/camera_info";

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&StereoNode::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
}

StereoNode::~StereoNode () {
    delete left_sub_;
    delete right_sub_;
    delete sync_;
}

void StereoNode::ImageCallback (const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr& msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgLeft->header.stamp;

  orb_slam_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, rclcpp::Time(msgLeft->header.stamp).seconds());

  Update ();
}
