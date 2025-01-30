from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("pointcloud_to_geotiff"), "config", "params.yaml"]
    )

    pointcloud_to_geotiff = Node(
        package="pointcloud_to_geotiff",
        executable="pointcloud_converter",
        name="pointcloud_converter",
        parameters=[params_file],
        output="screen",
    )

    slam_params_file = PathJoinSubstitution(
        [
            FindPackageShare("multi_session_slam"),
            "param",
            "multi_session_slam.yaml",
        ]
    )

    scan_matcher_node = Node(
        package="multi_session_slam",
        executable="scan_matcher_node",
        name="scan_matcher",
        parameters=[slam_params_file],
        remappings=[("/input_cloud", "/multiBeam")],
        output="screen",
    )

    multi_session_slam_node = Node(
        package="multi_session_slam",
        executable="multi_session_slam_node",
        name="multi_session_slam",
        parameters=[slam_params_file],
        remappings=[],
        output="screen",
    )

    return LaunchDescription(
        [
            pointcloud_to_geotiff,
            scan_matcher_node,
            multi_session_slam_node,
        ]
    )
