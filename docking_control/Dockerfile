FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# The controller uses Nav2 action messages and runs as a standalone ROS node.
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY docking_node.py .

CMD ["python3", "docking_node.py"]