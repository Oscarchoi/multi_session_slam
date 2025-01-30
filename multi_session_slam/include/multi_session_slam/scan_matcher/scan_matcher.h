// This software is licensed under the BSD 2-Clause License.
// Copyright (c) 2020, Ryohei Sasaki. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted under the following conditions:
//
// 1. Redistributions of source code must retain the copyright notice,
//    license conditions, and disclaimer.
// 2. Redistributions in binary form must reproduce the above information in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
// For full license details, see the LICENSE file in the project root.

#ifndef MULTI_SESSION_SLAM_SCAN_MATCHER_SCAN_MATCHER_H_
#define MULTI_SESSION_SLAM_SCAN_MATCHER_SCAN_MATCHER_H_

#include <atomic>
#include <deque>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "multi_session_slam_msgs/msg/point_cloud_with_pose.hpp"

namespace multi_session_slam {

class ScanMatcher : public rclcpp::Node {
 public:
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<PointType>;

  explicit ScanMatcher(const rclcpp::NodeOptions& options);
  ~ScanMatcher();

  void OnPointCloudReceived(
      const typename sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void UpdateAlignTransform(const Eigen::Matrix4f& transform);

 private:
  void PublishAdjustedPointCloud(PointCloudType::Ptr input_cloud,
                                 Eigen::Matrix4f pose,
                                 rclcpp::Time timestamp);
  PointCloudType::Ptr PreprocessInputCloud(
      const typename sensor_msgs::msg::PointCloud2::SharedPtr msg,
      geometry_msgs::msg::TransformStamped tf);
  PointCloudType::Ptr TransformPointCloud(
      const PointCloudType::Ptr& input_point,
      const Eigen::Matrix4f& transformation);

  pcl::Registration<PointType, PointType>::Ptr registration_;
  std::deque<PointCloudType::Ptr> cloud_queue_;
  Eigen::Vector3f previous_position_;
  Eigen::Matrix3f previous_rotation_;
  Eigen::Matrix4f current_align_matrix_;
  bool has_initial_cloud_ = false;
  bool is_map_updated_ = false;
  bool is_debug_ = false;

  rclcpp::Clock clock_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      input_cloud_subscription_;
  rclcpp::Publisher<multi_session_slam_msgs::msg::PointCloudWithPose>::SharedPtr
      modified_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_cloud_publisher_;

  // topic
  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string debug_cloud_topic_;
  std::string global_frame_id_;
  std::string robot_frame_id_;
  std::string odom_frame_id_;

  // parameters
  std::string registration_method_;
  double ndt_resolution_;
  int ndt_num_threads_;
  double gicp_corr_dist_threshold_;
  double trans_for_map_update_;
  double rotation_for_map_update_;
  double vg_size_for_input_;
  double vg_size_for_map_;
  bool use_min_max_filter_{false};
  double scan_min_range_{0.1};
  double scan_max_range_{100.0};
  double map_publish_period_;
  int num_targeted_cloud_;

  // initial_pose
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;
};

}  // namespace multi_session_slam

#endif  // MULTI_SESSION_SLAM_SCAN_MATCHER_SCAN_MATCHER_H_
