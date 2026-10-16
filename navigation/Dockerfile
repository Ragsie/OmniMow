FROM ros:jazzy-ros-base
# Nav2 for ROS 2 Jazzy (Ubuntu 24.04)
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY navigation.launch.py nav2_params.yaml /app/
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch /app/navigation.launch.py"]
