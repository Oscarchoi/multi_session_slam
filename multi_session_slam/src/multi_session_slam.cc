#include "multi_session_slam/multi_session_slam.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/voxel_grid_covariance_omp.h>

namespace multi_session_slam {

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

Eigen::Matrix4f convert(const geometry_msgs::msg::Pose& pose) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 3) = pose.position.x;
  transform(1, 3) = pose.position.y;
  transform(2, 3) = pose.position.z;

  Eigen::Quaternionf quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  return transform;
}

}  // namespace

MultiSessionSlam::MultiSessionSlam(const rclcpp::NodeOptions& options)
    : rclcpp::Node("multi_session_slam", options) {
  input_cloud_topic_ = get_or_default_parameter(this, "input_cloud_topic",
                                                std::string("/input_cloud"));
  output_cloud_topic_ = get_or_default_parameter(this, "output_cloud_topic",
                                                 std::string("/output_cloud"));
  session_start_service_ = get_or_default_parameter(
      this, "session_start_service", std::string("/session_start"));
  session_end_service_ = get_or_default_parameter(this, "session_end_service",
                                                  std::string("/session_end"));
  global_frame_id_ =
      get_or_default_parameter(this, "global_frame_id", std::string("map"));
  vg_size_for_input_ = get_or_default_parameter(this, "vg_size_for_input", 0.2);
  debug_ = get_or_default_parameter(this, "debug", false);

  auto registration_method =
      get_or_default_parameter(this, "registration_method", std::string("NDT"));
  auto voxel_leaf_size = get_or_default_parameter(this, "voxel_leaf_size", 0.2);
  auto threshold_loop_closure_score =
      get_or_default_parameter(this, "threshold_loop_closure_score", 1.0);
  auto range_of_searching_loop_closure =
      get_or_default_parameter(this, "range_of_searching_loop_closure", 20.0);
  auto loop_closure_search_num =
      get_or_default_parameter(this, "loop_closure_search_num", 10.0);
  auto num_adjacent_pose_constraints =
      get_or_default_parameter(this, "num_adjacent_pose_constraints", 5);

  if (debug_) {
    std::cout << "------------------" << std::endl;
    std::cout << "registration_method: " << registration_method << "\n"
              << "vg_size_for_input: " << vg_size_for_input_ << "\n"
              << "voxel_leaf_size: " << voxel_leaf_size << "\n"
              << "threshold_loop_closure_score: "
              << threshold_loop_closure_score << "\n"
              << "range_of_searching_loop_closure: "
              << range_of_searching_loop_closure << "\n"
              << "loop_closure_search_num: " << loop_closure_search_num << "\n"
              << "num_adjacent_pose_constraints: "
              << num_adjacent_pose_constraints << "\n"
              << "is_debug: " << debug_ << "\n";
    std::cout << "------------------" << std::endl;
  }

  input_cloud_subscription_ =
      create_subscription<multi_session_slam_msgs::msg::PointCloudWithPose>(
          input_cloud_topic_, rclcpp::SensorDataQoS(),
          std::bind(&MultiSessionSlam::OnPointCloudReceived, this,
                    std::placeholders::_1));
  output_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
  slam_session_start_service_ = create_service<test_msgs::srv::BasicTypes>(
      session_start_service_,
      std::bind(&MultiSessionSlam::OnSessionStartRequested, this,
                std::placeholders::_1, std::placeholders::_2));
  slam_session_end_service_ = create_service<test_msgs::srv::BasicTypes>(
      session_end_service_,
      std::bind(&MultiSessionSlam::OnSessionEndRequested, this,
                std::placeholders::_1, std::placeholders::_2));
}

MultiSessionSlam::~MultiSessionSlam() {}

void MultiSessionSlam::OnPointCloudReceived(
    const typename multi_session_slam_msgs::msg::PointCloudWithPose::SharedPtr
        msg) {
  RCLCPP_INFO(this->get_logger(), "Received pointcloud.");
  if (msg->cloud.header.frame_id != global_frame_id_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Received pointcloud with invalid frame: %s",
                 msg->cloud.header.frame_id);
    return;
  }

  PointCloudType::Ptr input_cloud(new PointCloudType);
  pcl::fromROSMsg(msg->cloud, *input_cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);

  PointCloudType::Ptr filtered_cloud(new PointCloudType);
  pcl::VoxelGrid<PointType> grid_filter;
  grid_filter.setLeafSize(vg_size_for_input_, vg_size_for_input_,
                          vg_size_for_input_);
  grid_filter.setInputCloud(input_cloud);
  grid_filter.filter(*filtered_cloud);
  filtered_cloud->is_dense = true;

  Eigen::Matrix4f pose = convert(msg->pose);

  size_t session_count = 0;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    for (auto& session : slam_sessions_) {
      session.second->RegisterPointCloud(filtered_cloud, pose);
    }
    session_count = slam_sessions_.size();
  }
  RCLCPP_INFO(this->get_logger(),
              "Received pointcloud (registered to %d sessions).",
              session_count);
}

void MultiSessionSlam::OnSessionStartRequested(
    const std::shared_ptr<test_msgs::srv::BasicTypes::Request> request,
    std::shared_ptr<test_msgs::srv::BasicTypes::Response> response) {
  const std::string session_key = request->string_value;
  RCLCPP_INFO(this->get_logger(), "Received SLAM session start requested: %s",
              session_key.c_str());

  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (slam_sessions_.find(session_key) != slam_sessions_.end()) {
      RCLCPP_WARN(this->get_logger(), "Session with key '%s' already exists.",
                  session_key.c_str());
      response->bool_value = false;
      return;
    }
    slam_sessions_[session_key] = std::make_shared<GraphSlam>(this);
  }

  RCLCPP_INFO(this->get_logger(), "SLAM session '%s' started successfully.",
              session_key.c_str());
  response->bool_value = true;
}

void MultiSessionSlam::OnSessionEndRequested(
    const std::shared_ptr<test_msgs::srv::BasicTypes::Request> request,
    std::shared_ptr<test_msgs::srv::BasicTypes::Response> response) {
  const std::string session_key = request->string_value;
  RCLCPP_INFO(this->get_logger(), "Received SLAM session end requested: %s",
              session_key.c_str());
  std::shared_ptr<GraphSlam> target_session;

  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    auto it = slam_sessions_.find(session_key);
    if (it == slam_sessions_.end()) {
      RCLCPP_WARN(this->get_logger(), "Session with key '%s' does not exist.",
                  session_key.c_str());
      response->bool_value = false;
      return;
    }
    target_session = it->second;  // copy from the map
    slam_sessions_.erase(it);
  }

  RCLCPP_INFO(this->get_logger(), "Start map generation for session '%s'...",
              session_key.c_str());
  // Wait for the session to be fully destroyed
  auto map = target_session->GenerateMapFromClouds();
  if (!map || map->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to generate map for session '%s'.",
                 session_key.c_str());
    response->bool_value = true;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Session '%s' finished successfully.",
              session_key.c_str());
  RCLCPP_INFO(this->get_logger(), "Generated pointcloud count: %d",
              map->size());

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*map, msg);
  msg.header.frame_id = global_frame_id_;
  msg.header.stamp = this->now();
  output_cloud_publisher_->publish(msg);

  response->bool_value = true;
}

}  // namespace multi_session_slam