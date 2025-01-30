#ifndef MULTI_SESSION_SLAM_MULTI_SESSION_SLAM_H_
#define MULTI_SESSION_SLAM_MULTI_SESSION_SLAM_H_

#include <map>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <test_msgs/srv/basic_types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "multi_session_slam/graph_slam/graph_slam.h"
#include "multi_session_slam_msgs/msg/point_cloud_with_pose.hpp"

namespace multi_session_slam {

class MultiSessionSlam : public rclcpp::Node {
 public:
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;

  explicit MultiSessionSlam(const rclcpp::NodeOptions& options);
  ~MultiSessionSlam();

 private:
  void OnPointCloudReceived(
      const typename multi_session_slam_msgs::msg::PointCloudWithPose::SharedPtr
          msg);
  void OnSessionStartRequested(
      const std::shared_ptr<test_msgs::srv::BasicTypes::Request> request,
      std::shared_ptr<test_msgs::srv::BasicTypes::Response> response);
  void OnSessionEndRequested(
      const std::shared_ptr<test_msgs::srv::BasicTypes::Request> request,
      std::shared_ptr<test_msgs::srv::BasicTypes::Response> response);

  std::map<std::string, std::shared_ptr<GraphSlam>> slam_sessions_;
  std::mutex session_mutex_;

  rclcpp::Subscription<multi_session_slam_msgs::msg::PointCloudWithPose>::
      SharedPtr input_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      output_cloud_publisher_;
  rclcpp::Service<test_msgs::srv::BasicTypes>::SharedPtr
      slam_session_start_service_;
  rclcpp::Service<test_msgs::srv::BasicTypes>::SharedPtr
      slam_session_end_service_;

  std::string input_cloud_topic_;
  std::string session_start_service_;
  std::string session_end_service_;

  std::string global_frame_id_;

  double vg_size_for_input_;
};

}  // namespace multi_session_slam

#endif  // MULTI_SESSION_SLAM_MULTI_SESSION_SLAM_H_