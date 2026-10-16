# libcamera from source: Ubuntu 24.04's libcamera 0.2 cannot run the Dragon Q6A's cameras on a
# mainline kernel (CAMSS + simple pipeline handler with the software ISP). Same version as the
# host (Ubuntu 26.04). EGL/GLES headers: libcamera builds the software ISP's GPU debayer
# (LIBCAMERA_SOFTISP_MODE=gpu in omnimow.env runs it on the Adreno GPU).
# libcamera/*.patch: lets both IMX219s stream at the same time (see the patch).
FROM ros:jazzy-ros-base AS libcamera
ARG LIBCAMERA_VERSION=v0.7.0
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ca-certificates meson ninja-build pkg-config g++ \
    python3-yaml python3-ply python3-jinja2 \
    libyaml-dev libssl-dev libudev-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libegl-dev libgles-dev \
    && rm -rf /var/lib/apt/lists/*
COPY libcamera/*.patch /patches/
RUN git clone --depth 1 -b "$LIBCAMERA_VERSION" https://github.com/libcamera-org/libcamera.git /src \
    && cd /src \
    && git apply /patches/*.patch \
    && meson setup build --prefix=/usr/local --buildtype=release \
        -Dpipelines=simple,uvcvideo -Dipas=simple -Dgstreamer=enabled -Dudev=enabled \
        -Dcam=disabled -Dqcam=disabled -Dlc-compliance=disabled -Dv4l2=disabled \
        -Dpycamera=disabled -Ddocumentation=disabled -Dtracing=disabled -Dtest=false \
    && ninja -C build -j2 \
    && DESTDIR=/out ninja -C build install

FROM ros:jazzy-ros-base

COPY --from=libcamera /out/usr/local /usr/local
# GStreamer finds libcamerasrc in /usr/local (path for arm64 and, for PC builds, amd64)
ENV GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:/usr/local/lib/x86_64-linux-gnu/gstreamer-1.0
# Debayer on the CPU unless omnimow.env says gpu. libcamera has no fallback: if the GPU debayer
# cannot start, the cameras do not open.
ENV LIBCAMERA_SOFTISP_MODE=cpu

# Combine all system packages into a single layer and clean up to keep the image small and fast
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-opencv \
    python3-yaml \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-cv-bridge \
    gstreamer1.0-tools \
    v4l-utils \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    libyaml-0-2 \
    libssl3t64 \
    libegl1 \
    libgles2 \
    libgl1 \
    libegl-mesa0 \
    libgl1-mesa-dri \
    libgbm1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/* \
    && ldconfig

WORKDIR /app
COPY . /app/

# Make the startup script executable
RUN chmod +x /app/start_vision.sh

# Run the startup script instead of launching python directly
CMD ["/app/start_vision.sh"]
