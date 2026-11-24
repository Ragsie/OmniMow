FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# The controller uses Nav2 action messages and TF (its pose on the map) and runs as a standalone ROS node.
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-nav2-msgs \
    ros-jazzy-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY docking_node.py .

CMD ["python3", "docking_node.py"]