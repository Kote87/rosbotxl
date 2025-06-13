FROM ros:humble
RUN apt-get update && apt-get install -y python3-numpy
WORKDIR /scan_filter
COPY front_cone.py .
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CMD ["python3","front_cone.py"]
