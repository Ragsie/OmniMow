FROM ros:jazzy-ros-base
# Visual SLAM: RTAB-Map stereo odometry + loop closure (map -> odom)
RUN apt-get update && apt-get install -y \
    ros-jazzy-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY slam.launch.py /app/
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch /app/slam.launch.py"]
