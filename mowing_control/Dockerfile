FROM ros:jazzy-ros-base
# Mowing mission controller: coverage planning (shapely), Nav2 actions, blade control
RUN apt-get update && apt-get install -y \
    python3-numpy \
    python3-shapely \
    ros-jazzy-nav2-msgs \
    ros-jazzy-tf2-ros-py \
    ros-jazzy-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY mowing_node.py coverage.py edge_follow.py /app/
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 /app/mowing_node.py"]
