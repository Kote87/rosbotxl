from setuptools import setup
import os

package_name = 'scan_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            os.path.join("share/ament_index/resource_index/packages"),
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools", "rclpy"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Lidar front-cone filter for ROSbot XL",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "front_cone = scan_filter.front_cone:main",
        ],
    },
)
