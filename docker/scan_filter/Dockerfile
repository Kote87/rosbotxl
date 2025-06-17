FROM ros:humble-ros-base

# System dependencies
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
COPY ws/src/scan_filter ./scan_filter

WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

ENV ROS_WS=/ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "scan_filter", "front_cone"]
