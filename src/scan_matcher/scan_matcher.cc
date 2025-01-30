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

#include "multi_session_slam/scan_matcher/scan_matcher.h"

#include <chrono>
#include <deque>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace multi_session_slam {

using namespace std::chrono_literals;

namespace {

template <typename T>
T get_or_default_parameter(rclcpp::Node* node,
                           const std::string& param_name,
                           const T& default_value) {
  if (!node->has_parameter(param_name)) {
    node->declare_parameter<T>(param_name, default_value);
  }
  return node->get_parameter(param_name).get_value<T>();
}

}  // namespace

ScanMatcher::ScanMatcher(const rclcpp::NodeOptions& options)
    : rclcpp::Node("scan_matcher"),
      current_align_matrix_(Eigen::Matrix4f::Identity()),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  // retreive topic names
  input_cloud_topic_ = get_or_default_parameter(this, "input_cloud_topic",
                                                std::string("input_cloud"));
  output_cloud_topic_ = get_or_default_parameter(this, "output_cloud_topic",
                                                 std::string("modified_cloud"));
  debug_cloud_topic_ = get_or_default_parameter(this, "debug_cloud_topic",
                                                std::string("target_cloud"));

  // retreive parameters
  global_frame_id_ =
      get_or_default_parameter(this, "global_frame_id", std::string("map"));
  robot_frame_id_ = get_or_default_parameter(this, "robot_frame_id",
                                             std::string("base_link"));
  registration_method_ =
      get_or_default_parameter(this, "registration_method", std::string("NDT"));

  ndt_resolution_ = get_or_default_parameter(this, "ndt_resolution", 5.0);
  ndt_num_threads_ = get_or_default_parameter(this, "ndt_num_threads", 0);
  gicp_corr_dist_threshold_ =
      get_or_default_parameter(this, "gicp_corr_dist_threshold", 5.0);

  trans_for_map_update_ =
      get_or_default_parameter(this, "trans_for_map_update", 1.5);
  rotation_for_map_update_ =
      get_or_default_parameter(this, "rotation_for_map_update", 0.2);
  is_debug_ = get_or_default_parameter(this, "debug", false);
  vg_size_for_input_ = get_or_default_parameter(this, "vg_size_for_input", 0.2);
  vg_size_for_map_ = get_or_default_parameter(this, "vg_size_for_map", 0.1);
  scan_min_range_ = get_or_default_parameter(this, "scan_min_range", 0.1);
  scan_max_range_ = get_or_default_parameter(this, "scan_max_range", 100.0);
  map_publish_period_ =
      get_or_default_parameter(this, "map_publish_period", 15.0);
  use_min_max_filter_ =
      get_or_default_parameter(this, "use_min_max_filter", false);
  num_targeted_cloud_ =
      get_or_default_parameter(this, "num_targeted_cloud", 10);
  if (num_targeted_cloud_ < 1) {
    std::cout << "num_tareged_cloud should be positive" << std::endl;
    num_targeted_cloud_ = 1;
  }

  if (is_debug_) {
    std::cout << "------------------" << std::endl;
    std::cout << "registration_method: " << registration_method_ << "\n"
              << "ndt_resolution[m]: " << ndt_resolution_ << "\n"
              << "ndt_num_threads: " << ndt_num_threads_ << "\n"
              << "gicp_corr_dist_threshold[m]: " << gicp_corr_dist_threshold_
              << "\n"
              << "trans_for_map_update[m]: " << trans_for_map_update_ << "\n"
              << "vg_size_for_input[m]: " << vg_size_for_input_ << "\n"
              << "vg_size_for_map[m]: " << vg_size_for_map_ << "\n"
              << "use_min_max_filter: " << std::boolalpha << use_min_max_filter_
              << "\n"
              << "scan_min_range[m]: " << scan_min_range_ << "\n"
              << "scan_max_range[m]: " << scan_max_range_ << "\n"
              << "num_targeted_cloud: " << num_targeted_cloud_ << std::endl;
    std::cout << "------------------" << std::endl;
  }

  if (registration_method_ == "NDT") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
        ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI,
                                                     pcl::PointXYZI>());
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(0.01);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(ndt_num_threads_);
    registration_ = ndt;
  } else {
    pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr gicp(
        new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
    gicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold_);
    gicp->setTransformationEpsilon(1e-8);
    registration_ = gicp;
  }

  input_cloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
          input_cloud_topic_, rclcpp::SensorDataQoS(),
          std::bind(&ScanMatcher::OnPointCloudReceived, this,
                    std::placeholders::_1));

  modified_cloud_publisher_ =
      create_publisher<multi_session_slam_msgs::msg::PointCloudWithPose>(
          output_cloud_topic_, 10);

  if (is_debug_) {
    debug_cloud_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(debug_cloud_topic_, 10);
  }
}

ScanMatcher::~ScanMatcher() {}

void ScanMatcher::OnPointCloudReceived(
    const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // received tranform
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(global_frame_id_, msg->header.frame_id,
                                    msg->header.stamp, rclcpp::Duration(0.0));
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(this->get_logger(), "Could not retrieve transform matrix: %s",
                e.what());
    return;
  }
  Eigen::Matrix4f sensor_transform =
      tf2::transformToEigen(tf.transform).matrix().cast<float>();

  // filter input pointcloud
  auto processed_cloud = PreprocessInputCloud(msg, tf);
  if (!processed_cloud) {
    RCLCPP_WARN(this->get_logger(), "Failed to handle received pointcloud.");
    return;
  }

  // set first target cloud if there is no initial cloud yet
  if (!has_initial_cloud_) {
    RCLCPP_INFO(get_logger(), "Received first pointcloud.");
    registration_->setInputTarget(processed_cloud);
    cloud_queue_.push_back(processed_cloud);
    PublishAdjustedPointCloud(processed_cloud, Eigen::Matrix4f::Identity(),
                              msg->header.stamp);

    previous_position_ = sensor_transform.block<3, 1>(0, 3);
    previous_rotation_ = sensor_transform.block<3, 3>(3, 3);
    current_align_matrix_ = Eigen::Matrix4f::Identity();
    has_initial_cloud_ = true;

    if (is_debug_ && debug_cloud_publisher_) {
      sensor_msgs::msg::PointCloud2 target_cloud_msg;
      pcl::toROSMsg(*processed_cloud, target_cloud_msg);
      target_cloud_msg.header.frame_id = global_frame_id_;
      target_cloud_msg.header.stamp = msg->header.stamp;
      debug_cloud_publisher_->publish(target_cloud_msg);
    }
    return;
  }

  // update target cloud if map updated
  if (is_map_updated_) {
    PointCloudType::Ptr target_cloud(new PointCloudType);
    for (auto& cloud : cloud_queue_) {
      *target_cloud += *cloud;
    }
    registration_->setInputTarget(target_cloud);
    is_map_updated_ = false;

    if (is_debug_ && debug_cloud_publisher_) {
      sensor_msgs::msg::PointCloud2 target_cloud_msg;
      pcl::toROSMsg(*target_cloud, target_cloud_msg);
      target_cloud_msg.header.frame_id = global_frame_id_;
      target_cloud_msg.header.stamp = msg->header.stamp;
      debug_cloud_publisher_->publish(target_cloud_msg);
    }
  }

  // register input cloud as source
  registration_->setInputSource(processed_cloud);
  PointCloudType::Ptr modified_cloud(new PointCloudType);
  registration_->align(*modified_cloud, current_align_matrix_);
  auto score = registration_->getFitnessScore();
  auto final_transform = registration_->getFinalTransformation();

  // check condition to update mapping
  auto modified_tf = final_transform * sensor_transform;
  auto modified_position = modified_tf.block<3, 1>(0, 3);
  auto translation = (modified_position - previous_position_).norm();
  auto modified_rotation = modified_tf.block<3, 3>(0, 0);
  auto relative_rotation = previous_rotation_.transpose() * modified_rotation;
  float rotation = std::acos((relative_rotation.trace() - 1.0f) / 2.0f);

  if (translation > trans_for_map_update_ ||
      std::fabs(rotation) > rotation_for_map_update_) {
    previous_position_ = modified_position;
    previous_rotation_ = modified_rotation;
    current_align_matrix_ = final_transform;

    // make the cloud queue to be sized queue
    if (cloud_queue_.size() >= static_cast<size_t>(num_targeted_cloud_)) {
      cloud_queue_.pop_front();
    }
    cloud_queue_.push_back(modified_cloud);

    PublishAdjustedPointCloud(modified_cloud, modified_tf, msg->header.stamp);
    is_map_updated_ = true;
  }

  if (is_debug_) {
    std::cout << "===============================" << std::endl;
    std::cout << "Fitness Score: " << score << std::endl;
    std::cout << "Translation: " << translation << std::endl;
    std::cout << "Rotation: " << rotation << std::endl;
    std::cout << "Cloud Queue Size: " << cloud_queue_.size() << std::endl;
  }
}

void ScanMatcher::PublishAdjustedPointCloud(PointCloudType::Ptr input_cloud,
                                            Eigen::Matrix4f pose,
                                            rclcpp::Time timestamp) {
  multi_session_slam_msgs::msg::PointCloudWithPose msg;
  pcl::toROSMsg(*input_cloud, msg.cloud);

  Eigen::Vector3f position(pose.block<3, 1>(0, 3));
  msg.pose.position.x = position[0];
  msg.pose.position.y = position[1];
  msg.pose.position.z = position[2];

  Eigen::Quaternionf quaternion(pose.block<3, 3>(0, 0));
  msg.pose.orientation.w = quaternion.w();
  msg.pose.orientation.x = quaternion.x();
  msg.pose.orientation.y = quaternion.y();
  msg.pose.orientation.z = quaternion.z();

  msg.cloud.header.frame_id = global_frame_id_;
  msg.cloud.header.stamp = timestamp;
  modified_cloud_publisher_->publish(msg);
}

ScanMatcher::PointCloudType::Ptr ScanMatcher::PreprocessInputCloud(
    const typename sensor_msgs::msg::PointCloud2::SharedPtr msg,
    geometry_msgs::msg::TransformStamped tf) {
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, tf);

  PointCloudType::Ptr input_cloud(new PointCloudType);
  pcl::fromROSMsg(transformed_cloud, *input_cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

  PointCloudType::Ptr filtered_cloud(new PointCloudType);
  pcl::VoxelGrid<PointType> grid_filter;
  grid_filter.setLeafSize(vg_size_for_input_, vg_size_for_input_,
                          vg_size_for_input_);
  grid_filter.setInputCloud(input_cloud);
  grid_filter.filter(*filtered_cloud);
  filtered_cloud->is_dense = true;
  return filtered_cloud;
}

ScanMatcher::PointCloudType::Ptr ScanMatcher::TransformPointCloud(
    const PointCloudType::Ptr& input_point,
    const Eigen::Matrix4f& transformation) {
  PointCloudType::Ptr transformed_cloud(new PointCloudType);
  pcl::transformPointCloud(*input_point, *transformed_cloud, transformation);
  return transformed_cloud;
}

}  // namespace multi_session_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(multi_session_slam::ScanMatcher)
