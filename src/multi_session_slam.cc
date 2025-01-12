#include "multi_session_slam/multi_session_slam.h"

#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

namespace ms_slam {

MultiSessionSlamNode::MultiSessionSlamNode(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options)
    : Node(node_name, node_options),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_) {
  declare_parameter("global_frame_id", "map");
  get_parameter("global_frame_id", global_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("input_cloud_topic", "input_cloud");
  get_parameter("input_cloud_topic", input_cloud_topic_);

  declare_parameter("vg_size_for_input", 0.2);
  get_parameter("vg_size_for_input", vg_size_for_input_);
  declare_parameter("vg_size_for_map", 0.1);
  get_parameter("vg_size_for_map", vg_size_for_map_);

  std::cout << "global_frame_id: " << global_frame_id_ << std::endl;
  std::cout << "robot_frame_id: " << robot_frame_id_ << std::endl;
  std::cout << "input_cloud_topic: " << input_cloud_topic_ << std::endl;
  std::cout << "-----------------" << std::endl;

  input_cloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
          input_cloud_topic_, rclcpp::SensorDataQoS(),
          std::bind(&MultiSessionSlamNode::OnInputCloudReceived, this,
                    std::placeholders::_1));
}

void MultiSessionSlamNode::OnInputCloudReceived(
    const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  sensor_msgs::msg::PointCloud2 transformed_msg;
  try {
    auto transform = tf_buffer_.lookupTransform(
        global_frame_id_, msg->header.frame_id, msg->header.stamp);
    tf2::doTransform(*msg, transformed_msg, transform);
  } catch (tf2::TransformException& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(transformed_msg, *pointcloud);

  // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  // voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  // voxel_grid.setInputCloud(pointcloud);
  // voxel_grid.filter(*pointcloud);
}

}  // namespace ms_slam