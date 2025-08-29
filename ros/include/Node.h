/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <orb_slam2_ros/srv/save_map.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>

#include "System.h"

class Node : public rclcpp::Node
{
  public:
    Node (ORB_SLAM2::System::eSensor sensor, const std::string& node_name);
    ~Node ();
    void Init ();

  protected:
    void Update ();
    ORB_SLAM2::System* orb_slam_;
    rclcpp::Time current_frame_time_;

    std::string camera_info_topic_;

  private:
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishGBAStatus (bool gba_status);
    void PublishRenderedImage (cv::Mat image);
    bool SaveMapSrv (const std::shared_ptr<orb_slam2_ros::srv::SaveMap::Request> request,
                     std::shared_ptr<orb_slam2_ros::srv::SaveMap::Response> response);
    void LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters);
    void DeclareParameters();

    // initialization Transform listener
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;

    tf2::Transform TransformFromMat (cv::Mat position_mat);
    tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher rendered_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_gba_publisher_;

    rclcpp::Service<orb_slam2_ros::srv::SaveMap>::SharedPtr service_server_;

    std::string name_of_node_;
    ORB_SLAM2::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;
};

#endif //ORBSLAM2_ROS_NODE_H_
