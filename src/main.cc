#include <rclcpp/rclcpp.hpp>

#include "multi_session_slam/multi_session_slam.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(std::make_shared<ms_slam::MultiSessionSlamNode>(
      "multi_session_slam", options));
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
