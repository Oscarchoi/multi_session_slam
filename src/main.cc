#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
