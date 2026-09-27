FROM ubuntu:24.04

# Keep package installation non-interactive during image builds.
ENV DEBIAN_FRONTEND=noninteractive

# Install basic Python environment and OpenCV
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    wget \
    software-properties-common \
    libgoogle-glog-dev \
    libc++-dev \
    && rm -rf /var/lib/apt/lists/*

# Expose the optional Qualcomm SNPE installation to the runtime and Python loader.
ENV SNPE_ROOT=/opt/qcom/snpe
ENV PATH="${SNPE_ROOT}/bin/x86_64-linux-clang:${PATH}"
ENV LD_LIBRARY_PATH="${SNPE_ROOT}/lib/x86_64-linux-clang:${SNPE_ROOT}/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="${SNPE_ROOT}/lib/python:${PYTHONPATH}"

# Set up and install ROS 2 Jazzy (standard on Ubuntu 24.04) or Humble libraries for ARM64
RUN add-apt-repository universe && \
    apt-get update && apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
    ros-jazzy-ros-core     ros-jazzy-cv-bridge     ros-jazzy-sensor-msgs     && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace/src/depth_fusion/

CMD ["python3", "/workspace/src/depth_fusion/depth_fusion_node.py"]