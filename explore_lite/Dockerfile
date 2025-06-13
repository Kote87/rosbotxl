FROM husarion/navigation2:humble-1.1.12-20240123

RUN apt-get update && apt-get install -y python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY explore_lite_src ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -rf /root/.colcon/ ~/.cache && \
    apt-get clean

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CMD ["bash","-c",". /opt/ros/humble/setup.bash && . /ws/install/setup.bash && \
      ros2 launch explore_lite explore.launch.py \
        planner_frequency:=1.0 visualize:=true progress_timeout:=60.0"]
