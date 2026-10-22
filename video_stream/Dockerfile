FROM ros:jazzy-ros-base
# WebRTC live video for the app (aiortc) - port 8889. aiortc is not packaged for Ubuntu 24.04:
# it goes into a venv that still sees the system's rclpy, OpenCV and NumPy.
ARG AIORTC_VERSION=1.14.0
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-opencv \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir aiortc==${AIORTC_VERSION} aiohttp
EXPOSE 8889
WORKDIR /app
COPY video_stream_node.py .
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec /opt/venv/bin/python3 video_stream_node.py"]
