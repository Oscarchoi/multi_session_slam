#include <rclcpp/rclcpp.hpp>

#include "multi_session_slam/multi_session_slam.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<multi_session_slam::MultiSessionSlam>(
      rclcpp::NodeOptions{});
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    5);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
