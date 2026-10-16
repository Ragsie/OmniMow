# Ubuntu 24.04 (noble) + ROS 2 Jazzy
FROM ros:jazzy-ros-base

ARG TARGETARCH
# Qualcomm QAI AppBuilder bundles the QNN runtime incl. the Hexagon v68 (QCS6490) libraries.
# Keep it on the same QAIRT version as the SDK that builds the .bin on the training server (or newer).
ARG QAI_APPBUILDER_VERSION=2.51.40

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/*

# arm64 (Radxa): QNN on the NPU. amd64 (PC): onnxruntime, so the node can be tested with a .onnx model.
RUN if [ "$TARGETARCH" = "arm64" ]; then \
        pip3 install --no-cache-dir --break-system-packages qai-appbuilder==${QAI_APPBUILDER_VERSION}; \
    else \
        pip3 install --no-cache-dir --break-system-packages onnxruntime; \
    fi

WORKDIR /app
COPY depth_fusion_node.py yolo_seg.py npu_model.py start_depth_fusion.sh /app/
RUN chmod +x /app/start_depth_fusion.sh

CMD ["/app/start_depth_fusion.sh"]
