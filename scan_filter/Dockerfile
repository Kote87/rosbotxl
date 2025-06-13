FROM ros:humble

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

RUN apt-get update && apt-get install -y python3-numpy
WORKDIR /scan_filter
COPY front_cone.py .
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

CMD ["ros2","run","scan_filter","front_cone.py"]

