FROM ros:jazzy-ros-base
# Keep the backend in a small ROS image because it only bridges telemetry and app requests.
# FastAPI/uvicorn from Ubuntu 24.04 (pip would need --break-system-packages on Python 3.12).
RUN apt-get update && apt-get install -y \
    python3-fastapi \
    python3-uvicorn \
    python3-websockets \
    && rm -rf /var/lib/apt/lists/*
EXPOSE 9000
WORKDIR /app
COPY backend_node.py .
CMD ["python3", "backend_node.py"]