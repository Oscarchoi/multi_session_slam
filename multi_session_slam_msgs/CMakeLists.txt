cmake_minimum_required(VERSION 3.5)
project(multi_session_slam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROSIDL dependencies
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(test_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointCloudWithPose.msg"
  "msg/PointCloudWithSessionInfo.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
