FROM husarion/navigation2:humble-1.1.12-20240123

# Adjustable parameters via environment variables
ENV FREQ="1.0" PROGRESS="60.0"

ENTRYPOINT ["ros2", "launch", "explore_lite", "explore.launch.py",
            "planner_frequency:=${FREQ}", "visualize:=true",
            "progress_timeout:=${PROGRESS}"]
