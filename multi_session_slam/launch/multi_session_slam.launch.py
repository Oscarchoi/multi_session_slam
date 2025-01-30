import os

import launch
import launch_ros.actions

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters_file = PathJoinSubstitution(
        [
            FindPackageShare("multi_session_slam"),
            "param",
            "multi_session_slam.yaml",
        ]
    )

    scan_matcher_node = launch_ros.actions.Node(
        package="multi_session_slam",
        executable="scan_matcher_node",
        name="scan_matcher",
        parameters=[parameters_file],
        remappings=[("/input_cloud", "/multiBeam")],
        output="screen",
    )

    multi_session_slam_node = launch_ros.actions.Node(
        package="multi_session_slam",
        executable="multi_session_slam_node",
        name="multi_session_slam",
        parameters=[parameters_file],
        remappings=[],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            scan_matcher_node,
            multi_session_slam_node,
        ]
    )
