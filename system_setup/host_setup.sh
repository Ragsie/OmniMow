#!/usr/bin/env bash
# OmniMow one-click installer for the Radxa Dragon Q6A (Qualcomm QCS6490) - TEST SETUP.
#
# Prepares the host and starts the whole ROS 2 stack in Docker. The desktop (GNOME) is left
# untouched, and the stack runs without the ESP32s; the micro-ROS agents are started as well
# once both ESP32s are connected and their udev serials are set.
#
# Tested images: rafaelwms/ubuntu-26.04.1-dragon-q6a (mainline kernel, FastRPC preinstalled),
# Radxa OS (Ubuntu 24.04). Safe to run again, e.g. after changing omnimow.env.
#
# From a clone of the repository:
#   sudo bash system_setup/host_setup.sh [--model FILE.bin] [--reboot]
# On a fresh board (clones the repository into ~/omnimow first):
#   curl -fsSL https://raw.githubusercontent.com/Ragsie/testing/test-setup/system_setup/host_setup.sh | sudo bash
#
#   --model FILE|URL  install this model instead of the newest release's (until a newer release).
#                  Normally not needed: the model comes from the newest GitHub release and is kept
#                  up to date automatically (update_model.sh, OMNIMOW_MODEL_* in omnimow.env).
#   --reboot       reboot when done (needed once for new groups and the camera overlay)
#   --user NAME    account to set up (default: found automatically - the one that ran sudo)
set -e

REPO_URL="https://github.com/Ragsie/testing.git"
REPO_BRANCH="test-setup"
MODEL_DIR=/opt/omnimow/models
MODEL_FILE="$MODEL_DIR/MowerAIn_seg_htp_context.bin"
LOG=/var/log/omnimow-install.log
export DEBIAN_FRONTEND=noninteractive

if [ "$EUID" -ne 0 ]; then
    echo "Run with root privileges: sudo bash host_setup.sh"
    exit 1
fi

MODEL_SRC=""
REBOOT=false
TARGET_USER=""
while [ $# -gt 0 ]; do
    case "$1" in
        --model)  MODEL_SRC="$2"; shift 2 ;;
        --reboot) REBOOT=true; shift ;;
        --user)   TARGET_USER="$2"; shift 2 ;;
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
        sudo -u "$TARGET_USER" git -C "$REPO_DIR" pull --ff-only < /dev/null
    else
        sudo -u "$TARGET_USER" git clone -b "$REPO_BRANCH" "$REPO_URL" "$REPO_DIR" < /dev/null
    fi
    args=(--user "$TARGET_USER")
    [ -n "$MODEL_SRC" ] && args+=(--model "$MODEL_SRC")
    [ "$REBOOT" = true ] && args+=(--reboot)
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
step "1/8 Configuration"
ENV_FILE="$SCRIPT_DIR/omnimow.env"
mkdir -p /etc/omnimow
ln -sf "$ENV_FILE" /etc/omnimow/omnimow.env
set -a
# shellcheck disable=SC1090
. "$ENV_FILE"
set +a
echo "Using $ENV_FILE; setting up user account '$TARGET_USER' ($TARGET_HOME)"

# ---------------------------------------------------------
# 2. PACKAGES
# ---------------------------------------------------------
step "2/8 Packages"
apt-get update
apt-get install -y curl git udev can-utils v4l-utils nfs-common device-tree-compiler

# FastRPC (NPU): keep what the image ships - installing the other family of packages on top
# (Qualcomm's fastrpc/libcdsprpc1 vs. Debian's libfastrpc1) conflicts and can break the NPU
if ldconfig -p | grep -q libcdsprpc; then
    echo "FastRPC already installed: $(ldconfig -p | grep -m1 libcdsprpc | awk '{print $NF}')"
elif apt-cache show libcdsprpc1 &>/dev/null; then
    install_available fastrpc libcdsprpc1 libadsprpc1
else
    install_available libfastrpc1 fastrpc-support hexagon-dsp-binaries
fi

# Host tools to check the cameras (the containers bring their own libcamera)
install_available libcamera-tools gstreamer1.0-libcamera gstreamer1.0-tools \
                  gstreamer1.0-plugins-base gstreamer1.0-plugins-good

# ---------------------------------------------------------
# 3. NPU ACCESS
# ---------------------------------------------------------
step "3/8 NPU (FastRPC) access"
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

# ---------------------------------------------------------
# 4. HARDWARE: CAMERAS, ESP32 SERIAL PORTS, PERFORMANCE
# ---------------------------------------------------------
step "4/8 Hardware"
# Stereo cameras: 2x IMX219 on CAM2 (right) and CAM3 (left), see camera/dual-imx219.dtsi.in
if [ -d /boot/efi/loader/entries ] && ls /boot/dtb-*/qcom/qcs6490-radxa-dragon-q6a.dtb &> /dev/null; then
    # systemd-boot image without device tree overlays (rafaelwms Ubuntu 26.04): build a board
    # .dtb with the cameras, boot it from its own entry, and redo it for every new kernel
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

# ModemManager probes new serial ports and gets in the way of the ESP32s
systemctl disable --now ModemManager.service 2>/dev/null || true

# Performance mode at every boot (the desktop stays as it is)
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
# 5. DOCKER
# ---------------------------------------------------------
step "5/8 Docker"
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
# 6. DATA DIRECTORIES AND MODEL
# ---------------------------------------------------------
step "6/8 Data directories and AI model"
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
        /usr/local/sbin/omnimow-update-model || warn "no model from the GitHub release ($OMNIMOW_MODEL_REPO) - publish a release with $OMNIMOW_MODEL_ASSET, or use --model" ;;
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
[ -f "$MODEL_FILE" ] || warn "No AI model at $MODEL_FILE - depth_fusion restarts until it is there (re-run with --model FILE.bin)"

if [ -n "$OMNIMOW_NFS_SERVER_IP" ]; then
    echo "Training server $OMNIMOW_NFS_SERVER_IP: mounting $OMNIMOW_NFS_SHARE on /opt/omnimow/incoming_raw"
    if ! grep -q "$OMNIMOW_NFS_SHARE" /etc/fstab; then
        echo "${OMNIMOW_NFS_SERVER_IP}:${OMNIMOW_NFS_SHARE} /opt/omnimow/incoming_raw nfs defaults,noauto,x-systemd.automount,x-systemd.device-timeout=10,_netdev,rw,nofail 0 0" >> /etc/fstab
    fi
    systemctl daemon-reload
    mount /opt/omnimow/incoming_raw || warn "NFS share could not be mounted"
fi

# ---------------------------------------------------------
# 7. BUILD AND START THE ROS 2 STACK
# ---------------------------------------------------------
step "7/8 Building and starting the ROS 2 stack (takes a while the first time)"
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

# ---------------------------------------------------------
# 8. STATUS
# ---------------------------------------------------------
step "8/8 Status"
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
if [ "$REBOOT" = true ]; then
    echo "Rebooting in 10 s ..."
    sleep 10
    reboot
else
    echo "Reboot once so $TARGET_USER gets the new groups (docker, fastrpc, video) and the camera"
    echo "overlay loads: sudo reboot. The stack starts by itself after the reboot."
fi
