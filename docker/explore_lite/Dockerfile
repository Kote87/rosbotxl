FROM husarion/navigation2:humble-1.1.12-20240123

RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
COPY ws/src/explore_lite ./explore_lite

WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -rf /root/.colcon/ ~/.cache && \
    apt-get clean

ENV ROS_WS=/ws
COPY docker/ros_ws_entrypoint.sh /ros_ws_entrypoint.sh
ENTRYPOINT ["/ros_ws_entrypoint.sh"]
CMD ["ros2", "launch", "explore_lite", "explore.launch.py", \
     "planner_frequency:=1.0", "visualize:=true", "progress_timeout:=60.0"]
