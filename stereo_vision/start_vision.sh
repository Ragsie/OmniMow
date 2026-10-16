#!/bin/bash
# --- STARTUP SCRIPT FOR STEREO VISION NODE ---
# First boot (or after "recalibrate"): run the stereo calibration until it succeeds,
# then start the stereo node with the rectified pipeline.
export OMNIMOW_CALIB_DIR=${OMNIMOW_CALIB_DIR:-/calibration}
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash

while [ ! -f "$OMNIMOW_CALIB_DIR/left.yaml" ] || [ ! -f "$OMNIMOW_CALIB_DIR/right.yaml" ]; do
    echo "No stereo calibration found in $OMNIMOW_CALIB_DIR - starting calibration."
    echo "Hold a checkerboard in front of both cameras (see calibrate_stereo.py)."
    if ! python3 /app/calibrate_stereo.py --out "$OMNIMOW_CALIB_DIR"; then
        echo "Calibration failed (reason in $OMNIMOW_CALIB_DIR/status.json) - retrying in 30 s"
        sleep 30
    fi
done

echo "Starting ROS 2 Launch sequence..."
exec ros2 launch /app/launch/mower.launch.py
