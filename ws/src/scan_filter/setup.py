from setuptools import setup
import os

package_name = 'scan_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share/ament_index/resource_index/packages'),
         ['resource/scan_filter']),
        (os.path.join('share/ament_index/resource_index/ros2_executable'),
         ['resource/scan_filter__front_cone']),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'front_cone = scan_filter.front_cone:main',
        ],
    },
)
