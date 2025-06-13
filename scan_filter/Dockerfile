FROM ros:humble
RUN apt-get update && apt-get install -y python3-numpy
WORKDIR /scan_filter
COPY front_cone.py .
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

CMD ["ros2","run","scan_filter","front_cone.py"]

