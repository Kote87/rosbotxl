FROM ros:humble

# ---------- PARCHE: clave nueva para packages.ros.org (jun-2024) ----------
RUN set -e ; \
    rm -f /etc/apt/sources.list.d/ros2-*.list ; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg ; \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list ; \
    apt-get update
# --------------------------------------------------------------------------

RUN apt-get install -y python3-numpy
WORKDIR /scan_filter
COPY front_cone.py .
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

CMD ["ros2","run","scan_filter","front_cone.py"]

