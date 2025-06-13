# Use the same base as the Nav2 container
FROM husarion/navigation2:humble-1.1.12-20240123


# 1. Additional build-time dependencies
RUN apt-get install -y --no-install-recommends \
    ros-humble-navigation2-msgs \
 && rm -rf /var/lib/apt/lists/*

# 2. Workspace
WORKDIR /ws
COPY explore_lite_src ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -rf /root/.colcon/ ~/.cache && \
    apt-get clean

# 3. Launch
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CMD ["bash","-c",". /opt/ros/humble/setup.bash && . /ws/install/setup.bash && \
      ros2 launch explore_lite explore.launch.py \
        planner_frequency:=1.0 visualize:=true progress_timeout:=60.0"]
