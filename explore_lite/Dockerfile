# Use the same base as the Nav2 container
FROM husarion/navigation2:humble-1.1.12-20240123

# --- refresco de clave ROS2 2024-06 ---
RUN sed -i 's|^deb .*packages.ros.org.*|# &|' /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/ros2.list && \
    apt-get update
# --------------------------------------

# 1. Additional build-time dependencies
RUN apt-get update && apt-get install -y \
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
