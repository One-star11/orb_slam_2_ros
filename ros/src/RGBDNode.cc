#include "RGBDNode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc > 1) {
        RCLCPP_WARN(rclcpp::get_logger("RGBDNode"), "Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto node = std::make_shared<RGBDNode>(ORB_SLAM2::System::RGBD, "RGBD");

    node->Init();

    rclcpp::spin(node);

    return 0;
}

RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, const std::string& node_name) 
  : Node (sensor, node_name) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (this, "/camera/rgb/image_raw");
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (this, "/camera/depth_registered/image_raw");
  camera_info_topic_ = "/camera/rgb/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(std::bind(&RGBDNode::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
}

RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}

void RGBDNode::ImageCallback (const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB, const sensor_msgs::msg::Image::ConstSharedPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, rclcpp::Time(msgRGB->header.stamp).seconds());

  Update ();
}
