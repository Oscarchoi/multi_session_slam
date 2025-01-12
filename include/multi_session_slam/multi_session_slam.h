#ifndef MULTI_SESSION_SLAM_MULTI_SESSION_SLAM_H_
#define MULTI_SESSION_SLAM_MULTI_SESSION_SLAM_H_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ms_slam {

class MultiSessionSlamNode : public rclcpp::Node {
 public:
  MultiSessionSlamNode(const std::string& node_name,
                       const rclcpp::NodeOptions& node_options);

 private:
  void OnInputCloudReceived(
      const typename sensor_msgs::msg::PointCloud2::SharedPtr msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      input_cloud_subscription_;

  std::string global_frame_id_;
  std::string robot_frame_id_;
  std::string input_cloud_topic_;

  double vg_size_for_map_;
  double vg_size_for_input_;
};

}  // namespace ms_slam

#endif  // MULTI_SESSION_SLAM_MULTI_SESSION_SLAM_H_