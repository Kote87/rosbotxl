#!/usr/bin/env python3
from setuptools import setup, find_packages

package_name = "scan_filter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools", "rclpy", "numpy"],
    zip_safe=True,
    maintainer="Maintainer",
    description="Front-cone LiDAR filter for ROSbot XL",
    license="Apache-2.0",
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            "share/ament_index/resource_index/ros2run",
            [f"resource/ros2run/{package_name}"],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    entry_points={
        "console_scripts": ["front_cone = scan_filter.front_cone:main"],
    },
)
