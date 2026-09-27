#!/usr/bin/env bash
# OmniMow host setup for Radxa Dragon Q6A (Qualcomm QCS6490, Ubuntu 24.04 LTS / Radxa OS R2)
# Includes system optimization: Headless configuration and GUI/Snap removal.
set -e

# Security check: Must be run with root privileges (sudo)
if [ "$EUID" -ne 0 ]; then
  echo "This script must be run with root privileges. Try: sudo ./omnimow_setup.sh"
  exit 1
fi

echo "=== Initializing OmniMow Host Setup & Optimization for Radxa Dragon Q6A ==="

# ---------------------------------------------------------
# 1. ENVIRONMENT & CONFIGURATION
# ---------------------------------------------------------
ENV_FILE="$(dirname "$0")/omnimow.env"
if [ ! -f "$ENV_FILE" ]; then
    echo "No omnimow.env found. Creating default configuration..."
    mkdir -p /etc/omnimow
    
    tee "$ENV_FILE" > /dev/null << 'EOF'
OMNIMOW_TRACK_WIDTH=0.40
OMNIMOW_ROBOT_RADIUS=0.28
OMNIMOW_NAV2_INFLATION_RADIUS=0.48

OMNIMOW_VESC_LEFT_ID=1
OMNIMOW_VESC_RIGHT_ID=2

# --- 3D STEREO MIPI CSI CAMERA SPECIFICATIONS ---
OMNIMOW_CAMERA_BASELINE=0.06
OMNIMOW_CAMERA_FOCAL_LENGTH=350.0

# --- SENSOR POSITIONS (URDF OFFSETS IN METERS) ---
OMNIMOW_CAMERA_HEIGHT_Z=0.10
OMNIMOW_CAMERA_OFFSET_X=0.25
OMNIMOW_CAMERA_PITCH_Y=0.0
OMNIMOW_GPS_OFFSET_X=-0.15
OMNIMOW_GPS_HEIGHT_Z=0.25

# MIPI CSI camera devices
OMNIMOW_CAMERA_DEVICE_LEFT="/dev/video0"
OMNIMOW_CAMERA_DEVICE_RIGHT="/dev/video1"

# --- MLOPS / LOCAL NFS AI TRAINING SERVER ---
OMNIMOW_NFS_SERVER_IP=""
OMNIMOW_NFS_SHARE="/mnt/nfs/omnimow_raw"
EOF
    ln -sf "$(realpath "$ENV_FILE")" /etc/omnimow/omnimow.env
fi

# Load variables into the environment
set -a
# shellcheck disable=SC1090
. "$ENV_FILE"
set +a

# ---------------------------------------------------------
# 2. CORE COMPONENTS INSTALLATION
# ---------------------------------------------------------
echo "Updating the system and installing core components..."
apt-get update
apt-get install -y curl git udev can-utils v4l-utils python3-pip nfs-common fastrpc libcdsprpc1 libadsprpc1

# Ensure FastRPC (NPU/DSP daemon) is running
systemctl enable --now fastrpc || true

# ---------------------------------------------------------
# 3. LOCK CRITICAL DRIVERS BEFORE CLEANUP
# ---------------------------------------------------------
echo "🔒 Locking critical Qualcomm and V4L2 packages..."
apt-mark hold fastrpc libcdsprpc1 libadsprpc1
apt-mark hold libv4l-0 v4l-utils libgstreamer1.0-0 || true

# ---------------------------------------------------------
# 4. UNINSTALL GUI AND BLOATWARE (STRIP SYSTEM)
# ---------------------------------------------------------
echo "🗑️ Removing desktop environment (GUI) and unnecessary applications..."
apt-get remove --purge -y gdm3 ubuntu-desktop* gnome-shell x11-common wayland-protocols libreoffice* thunderbird rhythmbox totem cups* pulseaudio*

echo "🗑️ Completely removing Snapd to save RAM and Disk space..."
apt-get remove --purge -y snapd
rm -rf /snap /var/snap /var/lib/snapd /var/cache/snapd

# Clean up
apt-get autoremove --purge -y
apt-get clean

# ---------------------------------------------------------
# 5. DISABLE HEAVY BACKGROUND SERVICES AND SET RUNLEVEL
# ---------------------------------------------------------
echo "🖥️ Changing default boot target to multi-user (Headless CLI)..."
systemctl set-default multi-user.target

echo "🛑 Stopping unnecessary services (ModemManager, Multipathd, Bluetooth)..."
services_to_disable=(
    "ModemManager.service"
    "multipathd.service"
    "multipathd.socket"
    "cups.service"
    "cups-browsed.service"
    "bluetooth.service"
)

for service in "${services_to_disable[@]}"; do
    systemctl stop "$service" 2>/dev/null || true
    systemctl disable "$service" 2>/dev/null || true
done

# ---------------------------------------------------------
# 6. DOCKER INSTALLATION
# ---------------------------------------------------------
if ! [ -x "$(command -v docker)" ]; then
    echo "Docker not found. Installing Docker Engine..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sh get-docker.sh
    # The default user on Radxa boards is usually 'radxa'
    usermod -aG docker radxa || true
    rm get-docker.sh
fi

# ---------------------------------------------------------
# 7. FILE SYSTEM, DIRECTORY STRUCTURE, AND NFS SETUP
# ---------------------------------------------------------
echo "Configuring data directories and file system for OmniMow..."
mkdir -p /opt/omnimow/models
mkdir -p /opt/omnimow/incoming_raw
chmod -R 777 /opt/omnimow

if [ ! -f /opt/omnimow/stats.json ]; then
    echo '{"total_distance_km": 0.0, "total_runtime_hours": 0.0}' > /opt/omnimow/stats.json
    chmod 777 /opt/omnimow/stats.json
fi

if [ -n "$OMNIMOW_NFS_SERVER_IP" ]; then
    echo "Local AI training server found ($OMNIMOW_NFS_SERVER_IP). Configuring systemd automount..."
    if ! grep -q "$OMNIMOW_NFS_SHARE" /etc/fstab; then
        echo "${OMNIMOW_NFS_SERVER_IP}:${OMNIMOW_NFS_SHARE} /opt/omnimow/incoming_raw nfs defaults,noauto,x-systemd.automount,x-systemd.device-timeout=10,_netdev,rw,nofail 0 0" >> /etc/fstab
    fi
    systemctl daemon-reload || true
    mount /opt/omnimow/incoming_raw || true
fi

# ---------------------------------------------------------
# 8. HARDWARE, OVERLAYS, AND PERFORMANCE
# ---------------------------------------------------------
echo "Enabling MIPI CSI device tree overlays (dual IMX219) via rsetup..."
if command -v rsetup &> /dev/null; then
    rsetup service enable overlay imx219-dual || true
    echo "Dual IMX219 MIPI CSI camera overlay enabled."
else
    echo "rsetup not found. Ensure dual IMX219 overlay is enabled manually."
fi

echo "Configuring udev rules for USB devices (ESP32, GPS)..."
tee /etc/udev/rules.d/99-omnimow.rules << 'EOF'
# ESP32 Micro-ROS Controller (CP2102 USB-to-UART)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyUSB_esp32", MODE="0666"

# Quectel LC29H RTK-GPS Receiver (Rover)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyUSB_gps_rover", MODE="0666"
EOF

udevadm control --reload-rules && udevadm trigger

echo "Setting Qualcomm QCS6490 CPU and NPU cores to Performance mode..."
for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo "performance" > "$governor" || true
done
echo "performance" > /sys/class/devfreq/*qcom,kgsl-3d0/governor || true

echo "=== System Setup & Optimization Complete! ==="
echo "A reboot is required to switch to Headless mode, load device tree overlays, and apply resource optimizations."
echo "Reboot now using: sudo reboot"