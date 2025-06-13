FROM ros:humble

# deps python
RUN apt-get update && apt-get install -y python3-numpy && apt-get clean

WORKDIR /scan_filter
COPY front_cone.py .
ENTRYPOINT ["python3", "/scan_filter/front_cone.py"]

