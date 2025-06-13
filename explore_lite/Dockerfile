FROM ros:humble
RUN apt-get update && apt-get install -y ros-humble-explore-lite \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CMD ["ros2", "launch", "explore_lite", "explore.launch.py"]
