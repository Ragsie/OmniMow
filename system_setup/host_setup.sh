#!/usr/bin/env bash
# OmniMow one-click installer for the Radxa Dragon Q6A (Qualcomm QCS6490) - TEST SETUP.
#
# Prepares the host and starts the whole ROS 2 stack in Docker. The stack runs without the ESP32s;
# the micro-ROS agents are started as well once both ESP32s are connected and their udev serials
# are set. To leave the board's RAM, CPU and disk to the stack, the host is trimmed: it boots
# without the desktop (GNOME stays installed, OMNIMOW_DESKTOP=true in robot.env brings it back),
# services the mower does not use are switched off, and unused packages are deleted after the build.
# Security updates keep installing themselves, and the Docker build cache is kept for fast rebuilds.
#
# The hardware is found automatically (OMNIMOW_PLATFORM / OMNIMOW_CAMERA_BACKEND=auto in omnimow.env):
#   Radxa Dragon Q6A  AI model on the NPU (QNN .bin), MIPI cameras through libcamera
#   x86 mini PC       AI model on the CPU (.onnx), USB cameras
#   Raspberry Pi      not tested: AI model on the CPU (.onnx), USB cameras only
# Two USB cameras are used on any board when they are plugged in. What was found is written to
# /etc/omnimow/hardware.env, which the containers read after omnimow.env.
#
# Tested images: rafaelwms/ubuntu-26.04.1-dragon-q6a (mainline kernel, FastRPC preinstalled),
# Radxa OS (Ubuntu 24.04). Safe to run again, e.g. after changing robot.env.
#
# Settings: omnimow.env holds the defaults (Worx Landroid M600 Plus) and is replaced by software
# updates. This mower's own values go in /etc/omnimow/robot.env, which updates never touch: on
# the first install from a terminal, configure_robot.sh asks for the robot's measurements.
#
# From a clone of the repository:
#   sudo bash system_setup/host_setup.sh [--model FILE.bin] [--reboot]
# On a fresh board (clones the repository into ~/omnimow first):
#   curl -fsSL https://raw.githubusercontent.com/Ragsie/OmniMow/main/system_setup/host_setup.sh | sudo bash
#
#   --model FILE|URL  install this model instead of the newest release's (until a newer release).
#                  Normally not needed: the model comes from the newest GitHub release and is kept
#                  up to date automatically (update_model.sh, OMNIMOW_MODEL_* in omnimow.env).
#                  The file type must suit the hardware: .bin on the Radxa NPU, .onnx elsewhere.
#   --reboot       reboot when done (needed once for new groups and the camera overlay)
#   --configure    ask for the robot's measurements again (configure_robot.sh)
#   --user NAME    account to set up (default: found automatically - the one that ran sudo)
set -e

REPO_URL="https://github.com/Ragsie/OmniMow.git"
REPO_BRANCH="main"
MODEL_DIR=/opt/omnimow/models
HW_ENV=/etc/omnimow/hardware.env
LOG=/var/log/omnimow-install.log
export DEBIAN_FRONTEND=noninteractive

if [ "$EUID" -ne 0 ]; then
    echo "Run with root privileges: sudo bash host_setup.sh"
    exit 1
fi

MODEL_SRC=""
REBOOT=false
TARGET_USER=""
CONFIGURE=false
while [ $# -gt 0 ]; do
    case "$1" in
        --model)  MODEL_SRC="$2"; shift 2 ;;
        --reboot) REBOOT=true; shift ;;
        --user)   TARGET_USER="$2"; shift 2 ;;
        --configure) CONFIGURE=true; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done
# Check the model file now rather than after half an hour of installing
case "$MODEL_SRC" in
    ""|http://*|https://*) ;;
    *)  if [ ! -f "$MODEL_SRC" ]; then
            echo "Model file not found: $MODEL_SRC"
            exit 1
        fi
        MODEL_SRC="$(realpath "$MODEL_SRC")" ;;
esac

# The user account that gets NPU, camera and Docker access: the one created on the board, not
# root or a leftover default account. In this order: --user, the account that ran sudo/pkexec,
# the account logged in on the desktop, the only regular account (UID >= 1000) there is.
human_users() {
    getent passwd | awk -F: '$3 >= 1000 && $3 < 60000 && $7 !~ /(nologin|false)$/ {print $1}'
}
detect_user() {
    local u
    for u in "$SUDO_USER" "$(id -nu "${PKEXEC_UID:-}" 2>/dev/null)" "$(logname 2>/dev/null)"; do
        [ -n "$u" ] && [ "$u" != root ] && { echo "$u"; return; }
    done
    # Logged in on the screen (GNOME session on seat0)
    for s in $(loginctl list-sessions --no-legend 2>/dev/null | awk '{print $1}'); do
        if [ "$(loginctl show-session "$s" -p Seat --value)" = seat0 ] && \
           [ "$(loginctl show-session "$s" -p Class --value)" = user ]; then
            u=$(loginctl show-session "$s" -p Name --value)
            [ -n "$u" ] && [ "$u" != root ] && { echo "$u"; return; }
        fi
    done
    [ "$(human_users | wc -l)" -eq 1 ] && human_users
}
[ -n "$TARGET_USER" ] || TARGET_USER="$(detect_user)"
if [ -z "$TARGET_USER" ] || ! id "$TARGET_USER" &>/dev/null; then
    if [ -n "$TARGET_USER" ]; then
        echo "User account '$TARGET_USER' does not exist."
    else
        echo "Could not tell which user account to set up."
    fi
    echo "Accounts on this board: $(human_users | tr '\n' ' ')"
    echo "Run again with: sudo bash host_setup.sh --user <name>"
    exit 1
fi
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"

# Piped from curl (no docker-compose.yml next to the script): clone the repository and run the
# installer from there
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" 2>/dev/null && pwd)"
if [ ! -f "$SCRIPT_DIR/docker-compose.yml" ]; then
    REPO_DIR="$TARGET_HOME/omnimow"
    # </dev/null: when piped from curl, stdin is the rest of this script
    apt-get update < /dev/null && apt-get install -y git < /dev/null
    if [ -d "$REPO_DIR/.git" ]; then
        # A software update from the app checks out a release tag (no branch to pull)
        sudo -u "$TARGET_USER" git -C "$REPO_DIR" pull --ff-only < /dev/null \
            || echo "Keeping the checkout in $REPO_DIR as it is"
    else
        sudo -u "$TARGET_USER" git clone -b "$REPO_BRANCH" "$REPO_URL" "$REPO_DIR" < /dev/null
    fi
    args=(--user "$TARGET_USER")
    [ -n "$MODEL_SRC" ] && args+=(--model "$MODEL_SRC")
    [ "$REBOOT" = true ] && args+=(--reboot)
    [ "$CONFIGURE" = true ] && args+=(--configure)
    exec bash "$REPO_DIR/system_setup/host_setup.sh" "${args[@]}" < /dev/null
fi

exec > >(tee -a "$LOG") 2>&1
echo "=== OmniMow test setup ($(date)) - log: $LOG ==="

step() { echo ""; echo "=== $* ==="; }
WARNINGS=()
warn() { echo "  ! $*"; WARNINGS+=("$*"); }

# Install each package this distribution has; skip the rest
install_available() {
    local pkg
    for pkg in "$@"; do
        if apt-cache show "$pkg" &>/dev/null; then
            apt-get install -y "$pkg" || warn "$pkg failed to install"
        else
            echo "  - $pkg not available here, skipped"
        fi
    done
}

# ---------------------------------------------------------
# 1. CONFIGURATION
# ---------------------------------------------------------
step "1/9 Configuration"
ENV_FILE="$SCRIPT_DIR/omnimow.env"
ROBOT_ENV=/etc/omnimow/robot.env
mkdir -p /etc/omnimow
ln -sf "$ENV_FILE" /etc/omnimow/omnimow.env
# Values changed by hand in omnimow.env (or merged into it by an older software update) move to
# robot.env, so the next update can replace omnimow.env
bash "$SCRIPT_DIR/configure_robot.sh" --migrate || warn "could not move your changes in omnimow.env to $ROBOT_ENV"
# The robot's measurements: asked once, on the first install from a terminal (also when piped
# from curl). Without a terminal (software update, cloud-init) the defaults stay.
if [ "$CONFIGURE" = true ] || [ ! -f "$ROBOT_ENV" ]; then
    if { : < /dev/tty; } 2> /dev/null; then
        bash "$SCRIPT_DIR/configure_robot.sh" < /dev/tty > /dev/tty 2>&1
    elif [ ! -f "$ROBOT_ENV" ]; then
        echo "No terminal: using the defaults of omnimow.env (Worx Landroid). Own robot: sudo bash $SCRIPT_DIR/host_setup.sh --configure"
    fi
fi
set -a
# shellcheck disable=SC1090
. "$ENV_FILE"
if [ -f "$ROBOT_ENV" ]; then
    # shellcheck disable=SC1090
    . "$ROBOT_ENV"
fi
set +a
echo "Using $ENV_FILE$([ -f "$ROBOT_ENV" ] && echo " and $ROBOT_ENV"); setting up user account '$TARGET_USER' ($TARGET_HOME)"

# Hardware: radxa (Dragon Q6A, NPU), pi or x86 (AI on the CPU), generic (other boards, as x86)
platform="${OMNIMOW_PLATFORM:-auto}"
if [ "$platform" = auto ]; then
    board=$(tr -d '\0' 2>/dev/null < /proc/device-tree/model || true)
    case "$board" in
        *"Dragon Q6A"*|*QCS6490*|*qcs6490*) platform=radxa ;;
        *"Raspberry Pi"*) platform=pi ;;
        *) if [ "$(uname -m)" = x86_64 ]; then platform=x86; else platform=generic; fi ;;
    esac
fi
# WSL (Windows): for testing on a PC. No desktop to switch off, and the app reaches the stack
# only with mirrored networking (see the warning at the end)
wsl=false
grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null && wsl=true
echo "Hardware: $platform (${board:-$(uname -m)})$([ "$wsl" = true ] && echo ", WSL")"
[ "$platform" = pi ] && warn "Raspberry Pi is not tested: the AI runs on the CPU and only USB cameras work"

# AI model: QNN context binary on the Radxa NPU, ONNX on the CPU everywhere else
if [ "$platform" = radxa ]; then
    model_asset="${OMNIMOW_MODEL_ASSET:-MowerAIn_seg_htp_context.bin}"
else
    model_asset="${OMNIMOW_MODEL_ASSET_ONNX:-MowerAIn_seg.onnx}"
fi
MODEL_FILE="$MODEL_DIR/$model_asset"
echo "AI model: $model_asset"

# Cameras: two USB cameras when they are plugged in, otherwise the Radxa's MIPI cameras. USB
# cameras are named by the USB port they are in (/dev/v4l/by-path), which stays the same between
# boots, unlike /dev/videoN.
camera_backend="${OMNIMOW_CAMERA_BACKEND:-auto}"
usb_cams=()
for path in /dev/v4l/by-path/*usb*-video-index0; do
    [ -e "$path" ] && usb_cams+=("$path")
done
if [ "$camera_backend" = auto ]; then
    if [ ${#usb_cams[@]} -ge 2 ]; then
        camera_backend=v4l2
    elif [ "$platform" = radxa ]; then
        camera_backend=libcamera
    else
        camera_backend=v4l2
        warn "found ${#usb_cams[@]} of 2 USB cameras - plug both in and run host_setup.sh again"
    fi
fi
echo "Cameras: $camera_backend (USB cameras found: ${#usb_cams[@]})"
{
    echo "# Written by host_setup.sh from the hardware it found - do not edit, set the values in"
    echo "# robot.env instead (they are only found automatically while set to auto there)."
    echo "OMNIMOW_PLATFORM=$platform"
    echo "OMNIMOW_CAMERA_BACKEND=$camera_backend"
    echo "OMNIMOW_MODEL_ASSET=$model_asset"
    echo "OMNIMOW_MODEL_PATH=/models/$model_asset"
    # Left/right by port order; if the calibration reports them swapped, swap the USB plugs
    if [ "$camera_backend" = v4l2 ] && [ ${#usb_cams[@]} -ge 2 ] && [ -z "$OMNIMOW_CAMERA_DEVICE_LEFT" ]; then
        echo "OMNIMOW_CAMERA_DEVICE_LEFT=${usb_cams[0]}"
        echo "OMNIMOW_CAMERA_DEVICE_RIGHT=${usb_cams[1]}"
    fi
} > "$HW_ENV"

# ---------------------------------------------------------
# 2. PACKAGES
# ---------------------------------------------------------
step "2/9 Packages"
apt-get update
# Only what the host itself needs: the containers bring their own tools (v4l-utils, GStreamer,
# libcamera ...). openssh-server: the board boots without the desktop (step 5).
apt-get install -y curl git udev openssh-server
[ -n "$OMNIMOW_NFS_SERVER_IP" ] && apt-get install -y nfs-common
# Tools earlier versions of this script installed but the host does not use: marked as
# automatically installed, so the autoremove at the end deletes them unless something needs them
for pkg in can-utils v4l-utils gstreamer1.0-libcamera gstreamer1.0-tools \
           gstreamer1.0-plugins-base gstreamer1.0-plugins-good; do
    dpkg -s "$pkg" &> /dev/null && apt-mark auto "$pkg" > /dev/null
done
[ -z "$OMNIMOW_NFS_SERVER_IP" ] && dpkg -s nfs-common &> /dev/null && apt-mark auto nfs-common > /dev/null

# FastRPC (NPU): keep what the image ships - installing the other family of packages on top
# (Qualcomm's fastrpc/libcdsprpc1 vs. Debian's libfastrpc1) conflicts and can break the NPU
if [ "$platform" != radxa ]; then
    echo "FastRPC (NPU) not needed on $platform"
elif ldconfig -p | grep -q libcdsprpc; then
    echo "FastRPC already installed: $(ldconfig -p | grep -m1 libcdsprpc | awk '{print $NF}')"
elif apt-cache show libcdsprpc1 &>/dev/null; then
    install_available fastrpc libcdsprpc1 libadsprpc1
else
    install_available libfastrpc1 fastrpc-support hexagon-dsp-binaries
fi

# 'cam -l' to check the MIPI cameras from the host (status at the end)
[ "$camera_backend" = libcamera ] && install_available libcamera-tools

# ---------------------------------------------------------
# 3. NPU ACCESS
# ---------------------------------------------------------
step "3/9 NPU (FastRPC) access"
if [ "$platform" = radxa ]; then
# The cDSP RPC daemon must run for the NPU (Hexagon HTP on the cDSP)
for unit in cdsprpcd adsprpcd; do
    if systemctl cat "$unit.service" &>/dev/null; then
        systemctl enable --now "$unit" || warn "$unit did not start (systemctl status $unit)"
    fi
done
# /dev/fastrpc-* and /dev/dma_heap/* without root, also over SSH (the image's own rule only
# covers the user logged in at the screen)
groupadd -f fastrpc
groupadd -f dmaheap
tee /etc/udev/rules.d/90-fastrpc.rules > /dev/null << 'EOF'
KERNEL=="fastrpc-*", GROUP="fastrpc", MODE="0660"
SUBSYSTEM=="dma_heap", GROUP="dmaheap", MODE="0660"
EOF
usermod -aG fastrpc,dmaheap,video,render "$TARGET_USER"
else
    echo "No NPU on $platform - the AI model runs on the CPU"
    usermod -aG video,render "$TARGET_USER" 2>/dev/null || usermod -aG video "$TARGET_USER"
fi

# ---------------------------------------------------------
# 4. HARDWARE: CAMERAS, ESP32 SERIAL PORTS, PERFORMANCE
# ---------------------------------------------------------
step "4/9 Hardware"
# Stereo cameras: 2x IMX219 on CAM2 (right) and CAM3 (left), see camera/dual-imx219.dtsi.in
if [ "$camera_backend" != libcamera ]; then
    echo "USB cameras - no camera device tree needed"
elif [ -d /boot/efi/loader/entries ] && ls /boot/dtb-*/qcom/qcs6490-radxa-dragon-q6a.dtb &> /dev/null; then
    # systemd-boot image without device tree overlays (rafaelwms Ubuntu 26.04): build a board
    # .dtb with the cameras, boot it from its own entry, and redo it for every new kernel
    apt-get install -y device-tree-compiler
    mkdir -p /usr/local/lib/omnimow
    rm -rf /usr/local/lib/omnimow/camera
    cp -r "$SCRIPT_DIR/camera" /usr/local/lib/omnimow/camera
    chmod 755 /usr/local/lib/omnimow/camera/install_camera_dtb.sh
    tee /etc/kernel/postinst.d/zz-omnimow-cameras > /dev/null << 'EOF'
#!/bin/sh
# OmniMow: camera boot entry for the new kernel (see host_setup.sh)
/usr/local/lib/omnimow/camera/install_camera_dtb.sh "$1" || true
EOF
    chmod 755 /etc/kernel/postinst.d/zz-omnimow-cameras
    /usr/local/lib/omnimow/camera/install_camera_dtb.sh --default \
        || warn "camera device tree could not be built - the cameras stay off"
    echo "If the board does not boot with the cameras: choose the normal entry in the boot menu,"
    echo "then run: sudo /usr/local/lib/omnimow/camera/install_camera_dtb.sh --remove"
elif command -v rsetup &> /dev/null; then
    # Radxa OS: Radxa's own overlays
    warn "enable the cameras in rsetup: Overlays -> 'Enable Radxa Camera 8M 219 on CAM2' and '... on CAM3'"
else
    warn "no way to enable the camera device tree on this image - see system_setup/camera/"
fi

# ESP32 micro-ROS controllers (CP2102 USB-UART), told apart by USB serial number. Find each serial
# with: udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' and replace CHANGE_ME_DRIVE/_CUTTER.
# Kept when it already exists so the serials entered there survive a re-run.
if [ ! -f /etc/udev/rules.d/99-omnimow.rules ]; then
    tee /etc/udev/rules.d/99-omnimow.rules > /dev/null << 'EOF'
# ESP32 Micro-ROS Controllers (CP2102 USB-to-UART).
# Both ESP32s have the same vendor/product ID, so they are told apart by USB serial number.
# Find each serial with: udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'
# and replace CHANGE_ME_DRIVE / CHANGE_ME_CUTTER below, then re-run host_setup.sh.
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="CHANGE_ME_DRIVE", SYMLINK+="ttyUSB_esp32_drive", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="CHANGE_ME_CUTTER", SYMLINK+="ttyUSB_esp32_cutter", MODE="0666"
EOF
fi
{ udevadm control --reload-rules && udevadm trigger; } || true

# Performance mode at every boot
tee /etc/systemd/system/omnimow-performance.service > /dev/null << 'EOF'
[Unit]
Description=OmniMow: CPU and GPU in performance mode

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor /sys/class/devfreq/*kgsl-3d0/governor; do [ -w "$$g" ] && echo performance > "$$g"; done; true'

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now omnimow-performance.service || warn "performance mode could not be set"

# ---------------------------------------------------------
# 5. TRIMMING THE HOST
# ---------------------------------------------------------
step "5/9 Trimming the host (desktop, unused services, logs)"
# No desktop: GNOME costs RAM and CPU the stack needs. Only the boot target changes (from the next
# boot on, so a running desktop session is not cut off); OMNIMOW_DESKTOP=true and a re-run bring it back.
if [ "$wsl" = true ]; then
    echo "WSL: no desktop to switch off"
elif [ "${OMNIMOW_DESKTOP:-false}" = true ]; then
    systemctl set-default graphical.target
    echo "Desktop on (OMNIMOW_DESKTOP=true)"
else
    systemctl set-default multi-user.target
    echo "Desktop off from the next boot (log in over SSH; OMNIMOW_DESKTOP=true in robot.env to keep it)"
fi

# Services the mower does not use. ModemManager also probes new serial ports and gets in the way of
# the ESP32s; power-profiles-daemon overrides the performance governor; snap refreshes download
# and install in the background while the mower runs. Unattended (security) upgrades stay on.
for unit in ModemManager.service \
            cups.service cups.socket cups.path cups-browsed.service \
            bluetooth.service \
            power-profiles-daemon.service switcheroo-control.service \
            packagekit.service fwupd.service fwupd-refresh.timer \
            apport.service whoopsie.service kerneloops.service \
            motd-news.timer \
            snapd.service snapd.socket snapd.seeded.service; do
    # Docker installed as a snap needs snapd
    case "$unit" in snapd*) snap list docker &> /dev/null && continue ;; esac
    if systemctl cat "$unit" &> /dev/null; then
        systemctl disable --now "$unit" &> /dev/null && echo "  - $unit off"
    fi
done
# Security updates: on (an earlier version of this script switched them off)
for unit in apt-daily.timer apt-daily-upgrade.timer unattended-upgrades.service; do
    systemctl cat "$unit" &> /dev/null && systemctl enable --now "$unit" &> /dev/null
done
systemctl enable --now ssh &> /dev/null || warn "SSH server did not start - without the desktop, log in on the console"

# Journal: at most 100 MB on disk
mkdir -p /etc/systemd/journald.conf.d
tee /etc/systemd/journald.conf.d/omnimow.conf > /dev/null << 'EOF'
[Journal]
SystemMaxUse=100M
EOF
systemctl restart systemd-journald
journalctl --vacuum-size=100M > /dev/null 2>&1 || true

# ---------------------------------------------------------
# 6. DOCKER
# ---------------------------------------------------------
step "6/9 Docker"
if ! command -v docker &> /dev/null; then
    # Docker's repository first; a brand-new release (Ubuntu 26.04) may not be in it yet
    if curl -fsSL https://get.docker.com -o /tmp/get-docker.sh && sh /tmp/get-docker.sh; then
        echo "Docker installed from download.docker.com"
    else
        echo "get.docker.com failed - using the distribution's Docker packages"
        apt-get install -y docker.io
        install_available docker-compose-v2 docker-buildx
    fi
    rm -f /tmp/get-docker.sh
fi
if ! docker compose version &> /dev/null; then
    install_available docker-compose-v2 docker-compose-plugin docker-compose
fi
docker compose version || { echo "'docker compose' is missing - cannot start the stack"; exit 1; }
systemctl enable --now docker
usermod -aG docker "$TARGET_USER"

# Container logs: rotate at 10 MB x 3 per container instead of growing until the disk is full
# (applies to newly created containers, so the stack is recreated once below)
recreate=()
if [ ! -f /etc/docker/daemon.json ]; then
    mkdir -p /etc/docker
    echo '{ "log-driver": "json-file", "log-opts": { "max-size": "10m", "max-file": "3" } }' > /etc/docker/daemon.json
    systemctl restart docker
    recreate=(--force-recreate)
elif ! grep -q '"max-size"' /etc/docker/daemon.json; then
    warn "/etc/docker/daemon.json exists without log rotation - add \"log-opts\": {\"max-size\": \"10m\", \"max-file\": \"3\"}"
fi
# A software update (update_software.sh) always restarts the containers on the images it just
# built: Compose does not always notice a rebuilt image and can leave the old container running
[ "${OMNIMOW_FORCE_RECREATE:-false}" = true ] && recreate=(--force-recreate)

# Portainer CE: web UI to watch and restart the containers and read their logs. Runs outside the
# OmniMow compose stack, so it keeps working while the stack is rebuilt or stopped.
if [ "${OMNIMOW_PORTAINER:-true}" = true ]; then
    portainer_port="${OMNIMOW_PORTAINER_PORT:-9443}"
    if docker container inspect portainer &> /dev/null; then
        echo "Portainer already installed: https://$(hostname -I | awk '{print $1}'):$portainer_port"
    else
        echo "Installing Portainer CE"
        docker volume create portainer_data > /dev/null
        if docker run -d --name portainer --restart=always \
                -p "$portainer_port:9443" \
                -v /var/run/docker.sock:/var/run/docker.sock \
                -v portainer_data:/data \
                portainer/portainer-ce:lts > /dev/null; then
            echo "Portainer: https://$(hostname -I | awk '{print $1}'):$portainer_port - create the admin user within 5 minutes"
        else
            warn "Portainer could not be started"
        fi
    fi
fi

# ---------------------------------------------------------
# 7. DATA DIRECTORIES AND MODEL
# ---------------------------------------------------------
step "7/9 Data directories and AI model"
mkdir -p "$MODEL_DIR" /opt/omnimow/incoming_raw /opt/omnimow/calibration /opt/omnimow/maps
[ -f /opt/omnimow/stats.json ] || \
    echo '{"total_distance_km": 0.0, "total_runtime_hours": 0.0}' > /opt/omnimow/stats.json
chmod -R 777 /opt/omnimow

# Automatic model updates from the newest GitHub release (see update_model.sh)
install -m 755 "$SCRIPT_DIR/update_model.sh" /usr/local/sbin/omnimow-update-model
tee /etc/systemd/system/omnimow-model-update.service > /dev/null << 'EOF'
[Unit]
Description=OmniMow: install a newer AI model from the latest GitHub release
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/omnimow-update-model
EOF
tee /etc/systemd/system/omnimow-model-update.timer > /dev/null << EOF
[Unit]
Description=OmniMow: check for a newer AI model

[Timer]
OnBootSec=2min
OnUnitActiveSec=${OMNIMOW_MODEL_CHECK_MIN:-60}min
RandomizedDelaySec=5min
Persistent=true

[Install]
WantedBy=timers.target
EOF
systemctl daemon-reload
systemctl enable omnimow-model-update.timer
systemctl restart omnimow-model-update.timer

case "$MODEL_SRC" in
    "")
        /usr/local/sbin/omnimow-update-model || warn "no model from the GitHub releases of $OMNIMOW_MODEL_REPO - publish a release with $model_asset (private repository: GitHub token in /etc/omnimow/github_token), or use --model" ;;
    http://*|https://*)
        echo "Downloading the model: $MODEL_SRC"
        if curl -fL --retry 3 -o "$MODEL_FILE.part" "$MODEL_SRC"; then
            mv -f "$MODEL_FILE.part" "$MODEL_FILE"
        else
            rm -f "$MODEL_FILE.part"
            warn "model download failed ($MODEL_SRC)"
        fi ;;
    *)  cp "$MODEL_SRC" "$MODEL_FILE.part" && mv -f "$MODEL_FILE.part" "$MODEL_FILE"
        echo "Model installed: $MODEL_SRC -> $MODEL_FILE" ;;
esac
# A hand-installed model is only replaced by a release published after it
if [ -n "$MODEL_SRC" ] && [ -f "$MODEL_FILE" ]; then
    /usr/local/sbin/omnimow-update-model --mark-current > /dev/null 2>&1 || true
fi
[ -f "$MODEL_FILE" ] || warn "No AI model at $MODEL_FILE - depth_fusion waits until it is there (re-run with --model FILE)"

[ -d "$SCRIPT_DIR/../.git" ] || warn "$(dirname "$SCRIPT_DIR") is not a git clone (a downloaded zip?) - software updates from the app need one: git clone $REPO_URL"

# Software updates: the app is told about a new GitHub release and installs it on request (see
# update_software.sh). The check runs on a timer; a request from the app (a file app_backend
# writes in /opt/omnimow) starts the check or the installation right away.
install -m 755 "$SCRIPT_DIR/update_software.sh" /usr/local/sbin/omnimow-update-software
tee /etc/systemd/system/omnimow-software-check.service > /dev/null << 'EOF'
[Unit]
Description=OmniMow: look for a new software release
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/omnimow-update-software --check
EOF
tee /etc/systemd/system/omnimow-software-check.timer > /dev/null << EOF
[Unit]
Description=OmniMow: look for a new software release

[Timer]
OnBootSec=3min
OnUnitActiveSec=${OMNIMOW_SOFTWARE_CHECK_MIN:-60}min
RandomizedDelaySec=5min
Persistent=true

[Install]
WantedBy=timers.target
EOF
tee /etc/systemd/system/omnimow-software-update.service > /dev/null << 'EOF'
[Unit]
Description=OmniMow: install a software release requested from the app
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
# Rebuilding the images can take a long time
TimeoutStartSec=infinity
ExecStart=/usr/local/sbin/omnimow-update-software --request
EOF
tee /etc/systemd/system/omnimow-software-update.path > /dev/null << 'EOF'
[Unit]
Description=OmniMow: watch for update requests from the app

[Path]
PathExists=/opt/omnimow/update_request
PathExists=/opt/omnimow/check_request

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable omnimow-software-check.timer omnimow-software-update.path
systemctl restart omnimow-software-check.timer
# Not restarted: when the app started this run, the update service is running it right now
# A path unit that hit its start limit (requests that failed again and again) stays off until reset
systemctl reset-failed omnimow-software-update.path omnimow-software-update.service 2> /dev/null || true
systemctl start omnimow-software-update.path
touch /opt/omnimow/check_request   # status for the app right away (after the update, if one runs)

if [ -n "$OMNIMOW_NFS_SERVER_IP" ]; then
    echo "Training server $OMNIMOW_NFS_SERVER_IP: mounting $OMNIMOW_NFS_SHARE on /opt/omnimow/incoming_raw"
    if ! grep -q "$OMNIMOW_NFS_SHARE" /etc/fstab; then
        echo "${OMNIMOW_NFS_SERVER_IP}:${OMNIMOW_NFS_SHARE} /opt/omnimow/incoming_raw nfs defaults,noauto,x-systemd.automount,x-systemd.device-timeout=10,_netdev,rw,nofail 0 0" >> /etc/fstab
    fi
    systemctl daemon-reload
    mount /opt/omnimow/incoming_raw || warn "NFS share could not be mounted"
fi

# ---------------------------------------------------------
# 8. BUILD AND START THE ROS 2 STACK
# ---------------------------------------------------------
step "8/9 Building and starting the ROS 2 stack (takes a while the first time)"
COMPOSE=(docker compose -f "$SCRIPT_DIR/docker-compose.yml")
# The micro-ROS agents (profile esp32) only when both ESP32s are connected
if [ -e /dev/ttyUSB_esp32_drive ] && [ -e /dev/ttyUSB_esp32_cutter ]; then
    echo "Both ESP32s found - starting the micro-ROS agents too"
    COMPOSE+=(--profile esp32)
else
    echo "ESP32s not connected - starting the stack without the micro-ROS agents"
fi

# Building the images (FusionCore, libcamera, RTAB-Map, Nav2 ...) needs more memory than the
# board's RAM when compilers run in parallel: add swap, and build one image at a time
mem_mb=$(awk '/MemTotal/ {print int($2 / 1024)}' /proc/meminfo)
swap_mb=$(awk '/SwapTotal/ {print int($2 / 1024)}' /proc/meminfo)
if [ "$mem_mb" -lt 12000 ] && [ "$swap_mb" -lt 4000 ] && [ ! -f /swapfile ]; then
    echo "Adding 8 GB swap (/swapfile) for building the images (RAM: ${mem_mb} MB)"
    if fallocate -l 8G /swapfile && chmod 600 /swapfile && mkswap /swapfile > /dev/null && swapon /swapfile; then
        grep -q '^/swapfile ' /etc/fstab || echo '/swapfile none swap sw 0 0' >> /etc/fstab
    else
        rm -f /swapfile
        warn "could not add swap - image builds may run out of memory"
    fi
fi

"${COMPOSE[@]}" pull --ignore-buildable || warn "some images could not be pulled"
# One image at a time: the compilers of parallel builds together exceed the memory. A failed
# image does not stop the others; only the services that built are started.
started=()
for svc in $("${COMPOSE[@]}" config --services); do
    if "${COMPOSE[@]}" config --format json | python3 -c 'import json,sys; sys.exit(0 if "build" in json.load(sys.stdin)["services"][sys.argv[1]] else 1)' "$svc"; then
        echo "--- Building $svc"
        if ! "${COMPOSE[@]}" build "$svc"; then
            warn "image $svc failed to build (see $LOG) - not started"
            continue
        fi
    fi
    started+=("$svc")
done
# All containers use 'restart: unless-stopped', so they come back by themselves after a reboot
if [ ${#started[@]} -gt 0 ] && ! "${COMPOSE[@]}" up -d --no-build "${recreate[@]}" "${started[@]}"; then
    warn "not all containers started - see: docker compose -f $SCRIPT_DIR/docker-compose.yml logs"
fi

# Free the disk: image versions replaced by this build, and the package cache and packages nothing
# needs any more. The build cache stays, so a rebuild only redoes what changed.
echo "Deleting replaced images and unused packages"
docker image prune -f > /dev/null || true
apt-get autoremove --purge -y || true
apt-get clean
df -h / | awk 'NR == 2 {print "Disk: " $4 " free of " $2}'

# ---------------------------------------------------------
# 9. STATUS
# ---------------------------------------------------------
step "9/9 Status"
for rp in /sys/class/remoteproc/remoteproc*; do
    [ -e "$rp/name" ] && echo "remoteproc $(cat "$rp/name"): $(cat "$rp/state")"
done
ls -l /dev/fastrpc-* 2>/dev/null || warn "No /dev/fastrpc-* - the NPU is not available"
if command -v cam > /dev/null; then
    cams=$(cam -l 2>/dev/null | grep -cE '^[0-9]+:' || true)
    echo "libcamera finds $cams camera(s)"
    [ "$cams" -ge 2 ] || warn "libcamera finds $cams of 2 cameras (check 'cam -l' after the reboot)"
fi
"${COMPOSE[@]}" ps

# WSL's default NAT network is only reachable from Windows itself, not from a phone
if [ "$wsl" = true ] && ! grep -qsi 'networkingMode *= *mirrored' /mnt/c/Users/*/.wslconfig; then
    warn "WSL: the app cannot reach the stack on WSL's NAT network ($(hostname -I | awk '{print $1}')) - turn on mirrored networking in %UserProfile%\.wslconfig (see Troubleshooting in the wiki)"
fi

echo ""
if [ ${#WARNINGS[@]} -gt 0 ]; then
    echo "Done, with warnings:"
    printf '  - %s\n' "${WARNINGS[@]}"
else
    echo "Done without warnings."
fi
echo "Log: $LOG"
docker container inspect portainer &> /dev/null && \
    echo "Portainer (containers, logs): https://$(hostname -I | awk '{print $1}'):${OMNIMOW_PORTAINER_PORT:-9443}"
echo "Logs of a container: docker compose -f $SCRIPT_DIR/docker-compose.yml logs -f depth_fusion"
if [ "$wsl" = true ]; then
    echo "WSL: restart it once (in Windows: wsl --shutdown) so $TARGET_USER gets the new groups."
elif [ "$REBOOT" = true ]; then
    echo "Rebooting in 10 s ..."
    sleep 10
    reboot
else
    echo "Reboot once so $TARGET_USER gets the new groups (docker, fastrpc, video), the camera"
    echo "overlay loads and the desktop stays off: sudo reboot. The stack starts by itself after the reboot."
fi
