FROM ros:humble-ros-base
WORKDIR /scan_filter
COPY front_cone.py .

ENTRYPOINT ["python3", "-u", "front_cone.py",
            "--center_deg", "106",
            "--fov_deg", "120"]

