FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /scan_filter
COPY front_cone.py .

CMD ["ros2","run","scan_filter","front_cone.py"]
