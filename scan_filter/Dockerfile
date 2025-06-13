FROM ros:humble

RUN apt-get update && apt-get install -y python3-numpy && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/scan_filter
COPY front_cone.py .

# Execute the filter directly without `ros2 run`
ENTRYPOINT ["python3", "/opt/scan_filter/front_cone.py", "--ros-args"]

