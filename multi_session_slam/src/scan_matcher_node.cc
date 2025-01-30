#include <rclcpp/rclcpp.hpp>

#include "multi_session_slam/scan_matcher/scan_matcher.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<multi_session_slam::ScanMatcher>(rclcpp::NodeOptions{});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
