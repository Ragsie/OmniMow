FROM ros:humble-ros-base
# Keep the backend in a small ROS image because it only bridges telemetry and app requests.
WORKDIR /app
COPY backend_node.py .
CMD ["python3", "backend_node.py"]