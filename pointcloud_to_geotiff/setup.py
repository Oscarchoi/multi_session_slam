from setuptools import setup

import os
from glob import glob

package_name = "pointcloud_to_geotiff"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oscarchoi",
    maintainer_email="wychoi502@gmail.com",
    description="A ROS 2 package that converts PointCloud2 data into GeoTIFF images for mapping and GIS applications.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pointcloud_converter = pointcloud_to_geotiff.pointcloud_converter:main",
        ],
    },
)
