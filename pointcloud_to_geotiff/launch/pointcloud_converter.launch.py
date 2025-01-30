from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("pointcloud_to_geotiff"), "config", "params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="pointcloud_to_geotiff",
                executable="pointcloud_converter",
                name="pointcloud_converter",
                parameters=[params_file],
                output="screen",
            )
        ]
    )
