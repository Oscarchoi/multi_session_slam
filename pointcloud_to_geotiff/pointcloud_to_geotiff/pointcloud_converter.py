import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import geometry_msgs.msg

import argparse
import datetime
import cv2
import numpy as np
import struct
import yaml
from osgeo import gdal, osr


class PointCloudConverter(Node):
    def __init__(self):
        super().__init__("pointcloud_converter")

        # default parameters
        self.declare_parameter("input_cloud_topic", "/output_cloud")
        self.input_cloud_topic = self.get_parameter("input_cloud_topic").value
        self.declare_parameter("output_directory", "/tmp/geotiff_maps")
        self.output_directory = self.get_parameter("output_directory").value

        # tf update parameters
        self.declare_parameter("enable_tf_update", True)
        self.enable_tf_update = self.get_parameter("enable_tf_update").value
        self.declare_parameter("global_frame_id", "map")
        self.global_frame_id = self.get_parameter("global_frame_id").value
        self.declare_parameter("robot_frame_id", "base_link")
        self.robot_frame_id = self.get_parameter("robot_frame_id").value

        # other parameters
        self.declare_parameter("dense_resolution", 0.05)
        self.dense_resolution = self.get_parameter("dense_resolution").value
        self.declare_parameter("sparse_resolution", 0.2)
        self.sparse_resolution = self.get_parameter("sparse_resolution").value
        self.resolution = self.sparse_resolution

        self.declare_parameter("sonar_distance", 80.0)
        self.sonar_distance = self.get_parameter("sonar_distance").value
        self.declare_parameter("initial_x", 0.0)
        self.initial_x = self.get_parameter("initial_x").value
        self.declare_parameter("initial_y", 0.0)
        self.initial_y = self.get_parameter("initial_y").value
        self.declare_parameter("enable_png", False)
        self.enable_png = self.get_parameter("enable_png").value
        self.declare_parameter("enable_metadata", False)
        self.enable_metadata = self.get_parameter("enable_metadata").value

        self.subscription = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self.listener_callback, 1
        )
        self.latest_subscription = rclpy.time.Time()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grid_size = int(self.sonar_distance * 2 / self.resolution)
        self.x = self.initial_x
        self.y = self.initial_y

        self.get_logger().info(f"----------------------")
        self.get_logger().info(f"input_cloud_topic: {self.input_cloud_topic}")
        self.get_logger().info(f"output_directory: {self.output_directory}")
        self.get_logger().info(f"enable_tf_update: {self.enable_tf_update}")
        self.get_logger().info(f"global_frame_id: {self.global_frame_id}")
        self.get_logger().info(f"robot_frame_id: {self.robot_frame_id}")
        self.get_logger().info(f"resolution: {self.resolution}")
        self.get_logger().info(f"sonar_distance: {self.sonar_distance}")
        self.get_logger().info(f"initial_x: {self.initial_x}")
        self.get_logger().info(f"initial_y: {self.initial_y}")
        self.get_logger().info(f"enable_png: {self.enable_png}")
        self.get_logger().info(f"enable_metadata: {self.enable_metadata}")
        self.get_logger().info(f"grid_size: {self.grid_size}x{self.grid_size} (WxH).")
        self.get_logger().info(f"----------------------")

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Received pointcloud ({int(msg.row_step/msg.point_step)} points)."
        )

        if self.enable_tf_update:
            self.get_base_link_pose(msg.header.stamp)

        points = self.pointcloud2_to_array(msg)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        mean_x = np.mean(x)
        mean_y = np.mean(y)
        mean_z = np.mean(z)
        self.get_logger().info(
            f"Mean Point: x={mean_x:.3f}, y={mean_y:.3f}, z={mean_z:.3f}"
        )
        threshold = self.grid_size / 2
        if abs(mean_x - self.x) > threshold or abs(mean_y - self.y) > threshold:
            self.get_logger().warn(f"Mean position is too far from initial position.")

        # filter points
        x_min = float(-self.sonar_distance) + self.x
        x_max = float(self.sonar_distance) + self.x
        y_min = float(-self.sonar_distance) + self.y
        y_max = float(self.sonar_distance) + self.y
        mask = (x_min <= x) & (x < x_max) & (y_min <= y) & (y < y_max)
        num_true_points = mask.sum()
        if num_true_points == 0:
            self.get_logger().error(
                f"No points in the point cloud were filtered as true within the grid area."
            )
            return
        else:
            self.get_logger().info(f"Filtered pointcloud: {num_true_points}.")

        filtered_x = x[mask]
        filtered_y = y[mask]
        filtered_z = z[mask]
        x_indices = np.floor((filtered_x - x_min) / self.resolution).astype(int)
        y_indices = np.floor((filtered_y - y_min) / self.resolution).astype(int)
        z_values = np.floor(filtered_z).astype(int)

        # update highest z value
        grid = np.full((self.grid_size, self.grid_size), float("-inf"))
        np.maximum.at(grid, (y_indices, x_indices), z_values)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        metadata = {
            "pixel_height": self.grid_size,
            "pixel_width": self.grid_size,
            "resolution": self.resolution,
        }

        if self.enable_metadata:
            with open(
                f"{self.output_directory}/metadata_{timestamp}.yaml", "w"
            ) as file:
                yaml.dump(metadata, file, default_flow_style=False)

        if self.enable_png:
            self.convert_to_png(grid, timestamp)

        self.convert_to_geotiff(grid, timestamp)

    def pointcloud2_to_array(self, cloud_msg):
        fmt = "ffff"  # x, y, z, intensity
        point_step = cloud_msg.point_step

        data = cloud_msg.data
        cloud_points = []
        for i in range(0, len(data), point_step):
            x, y, z, intensity = struct.unpack(fmt, data[i : i + point_step][:16])
            cloud_points.append([x, y, z, intensity])

        return np.array(cloud_points)

    def convert_to_png(self, grid, timestamp):
        valid_mask = grid > float("-inf")
        z_min = np.min(grid[valid_mask])
        z_max = np.max(grid)
        if z_max - z_min == 0:
            return

        z_scale = 1.0 / (z_max - z_min) * 255.0
        z_offset = z_min

        norm_grid = (grid - z_offset) * z_scale
        norm_grid[np.isneginf(grid)] = 0
        norm_grid = norm_grid.astype(np.uint8)
        norm_grid = norm_grid[:, ::-1]

        output_file = f"{self.output_directory}/pointcloud_{timestamp}.png"
        cv2.imwrite(output_file, norm_grid)

        self.get_logger().info(f"INFO: Saved pointcloud to '{output_file}'")

    def convert_to_geotiff(self, grid, timestamp):
        output_file = f"{self.output_directory}/output_{timestamp}.tiff"

        driver = gdal.GetDriverByName("GTiff")
        dataset = driver.Create(
            output_file, self.grid_size, self.grid_size, 1, gdal.GDT_Float32
        )

        data = grid[::-1, ::-1]
        data[np.isneginf(data)] = np.NaN
        band = dataset.GetRasterBand(1)
        band.WriteArray(data)

        band.SetNoDataValue(0.0)
        band.SetStatistics(
            np.nanmin(data), np.nanmax(data), np.nanmean(data), np.nanstd(data)
        )

        srs = osr.SpatialReference()
        srs.ImportFromEPSG(3857)  # 예: Pseudo Mercator
        dataset.SetProjection(srs.ExportToWkt())

        # FIXME: Use your own GeoTransform
        x_min = 0
        y_min = 0
        x_max = 1000
        y_max = 1000
        x_res = (x_max - x_min) / self.grid_size
        y_res = (y_max - y_min) / self.grid_size
        dataset.SetGeoTransform((x_min, x_res, 0, y_min, 0, -y_res))

        # close file
        dataset = None

        self.get_logger().info(f"INFO: Saved pointcloud to '{output_file}'")

    def get_base_link_pose(self, timestamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame_id,
                self.robot_frame_id,
                timestamp,
                timeout=rclpy.duration.Duration(seconds=3),
            )
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
        except tf2_ros.LookupException as e:
            self.get_logger().warn(
                "TF Lookup failed: Could not get %s pose: %s", self.robot_frame_id, e
            )
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(
                "TF Lookup failed: Could not get %s pose: %s", self.robot_frame_id, e
            )


def main(args=None):
    rclpy.init(args=args)
    pointcloud_converter = PointCloudConverter()

    try:
        rclpy.spin(pointcloud_converter)
    except KeyboardInterrupt:
        pointcloud_converter.get_logger().info(
            "KeyboardInterrupt detected. Shutting down cleanly..."
        )
    finally:
        pointcloud_converter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
