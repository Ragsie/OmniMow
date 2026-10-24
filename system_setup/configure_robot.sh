#!/usr/bin/env bash
# OmniMow robot settings: the values of this mower, kept apart from the repository.
#
# omnimow.env (in the repository) holds the defaults, for the Worx Landroid M600 Plus. A software
# update replaces it with the release's version. The values of this mower - its measurements, a
# different camera, a fork to update from, anything else from omnimow.env - go in
# /etc/omnimow/robot.env, which no update touches. robot.env is read after omnimow.env, so a value
# set there wins; settings it leaves out keep following the release.
#
#   configure_robot.sh                   ask for the robot's measurements (host_setup.sh runs this
#                                        on the first install when it has a terminal)
#   configure_robot.sh --set KEY=VALUE   set values in robot.env without questions (several allowed)
#   configure_robot.sh --show            what robot.env sets
#   configure_robot.sh --migrate         move values changed in the repository's omnimow.env to
#                                        robot.env and restore the file (host_setup.sh and
#                                        update_software.sh do this before an update)
#
# After a change: sudo bash system_setup/host_setup.sh (rebuilds and restarts what changed).
# The track width and VESC IDs are also built into the drive ESP32's firmware (platformio.ini):
# flash it again after changing them.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULTS="$SCRIPT_DIR/omnimow.env"
ROBOT_ENV="${OMNIMOW_ROBOT_ENV:-/etc/omnimow/robot.env}"

# set_values KEY=VALUE ...: replaces the key's line in robot.env, or appends it; comments stay
set_values() {
    mkdir -p "$(dirname "$ROBOT_ENV")"
    python3 - "$ROBOT_ENV" "$@" << 'EOF'
import os, sys
path, pairs = sys.argv[1], sys.argv[2:]
try:
    with open(path) as f:
        lines = f.read().splitlines()
except FileNotFoundError:
    lines = ["# OmniMow: the settings of this mower (configure_robot.sh). Read after omnimow.env,",
             "# so these win; software updates never change this file. After a change, run",
             "# sudo bash system_setup/host_setup.sh"]
for pair in pairs:
    key = pair.split("=", 1)[0]
    for i, line in enumerate(lines):
        if line.split("=", 1)[0].strip() == key:
            lines[i] = pair
            break
    else:
        lines.append(pair)
with open(path + ".tmp", "w") as f:
    f.write("\n".join(lines) + "\n")
os.chmod(path + ".tmp", 0o644)
os.replace(path + ".tmp", path)
EOF
}

# The settings in force: omnimow.env, then robot.env
load() {
    set -a
    # shellcheck disable=SC1090
    . "$DEFAULTS"
    if [ -f "$ROBOT_ENV" ]; then
        # shellcheck disable=SC1090
        . "$ROBOT_ENV"
    fi
    set +a
}

migrate() {
    local repo owner rel=system_setup/omnimow.env changed
    repo="$(dirname "$SCRIPT_DIR")"
    [ -d "$repo/.git" ] || return 0
    owner="$(stat -c %U "$repo")"
    g() { sudo -u "$owner" git -C "$repo" "$@"; }
    g diff --quiet HEAD -- "$rel" && return 0
    # KEY=VALUE lines that differ from the checked-out version (what was changed by hand, or by
    # an older software update that merged the user's changes into the file)
    mapfile -t changed < <(python3 - "$DEFAULTS" <(g show "HEAD:$rel") << 'EOF'
import sys
def parse(path):
    values = {}
    for line in open(path):
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            values[line.split("=", 1)[0].strip()] = line
    return values
mine, base = parse(sys.argv[1]), parse(sys.argv[2])
for key, line in mine.items():
    if base.get(key) != line:
        print(line)
EOF
)
    if [ ${#changed[@]} -gt 0 ]; then
        set_values "${changed[@]}"
        echo "Moved your settings from omnimow.env to $ROBOT_ENV:"
        printf '  %s\n' "${changed[@]}"
    fi
    g checkout --quiet -- "$rel"
}

# ask VAR "question" DEFAULT: the answer, or DEFAULT on Enter. A number for a number.
ask() {
    local answer number='^-?[0-9]+([.][0-9]+)?$'
    while true; do
        read -r -p "  $2 [$3]: " answer
        answer="${answer:-$3}"
        answer="${answer//,/.}"   # 0,40 as well as 0.40
        [[ "$3" =~ $number ]] && ! [[ "$answer" =~ $number ]] && { echo "  A number, please"; continue; }
        break
    done
    printf -v "$1" '%s' "$answer"
}

# m/rad <-> cm/degrees for the questions
calc() { python3 -c "import math; print(round($1, 4))"; }

wizard() {
    load
    # Current outline: x forward of the drive axle, y to the left
    read -r front rear width < <(python3 -c '
import json, sys
pts = json.loads(sys.argv[1])
print(round(max(p[0] for p in pts) * 100), round(-min(p[0] for p in pts) * 100),
      round((max(p[1] for p in pts) - min(p[1] for p in pts)) * 100))' "$OMNIMOW_FOOTPRINT")
    echo ""
    echo "OmniMow robot setup - saved in $ROBOT_ENV (software updates keep it)"
    echo "  1) Worx Landroid M600 Plus - the defaults in omnimow.env, nothing to measure"
    echo "  2) My own robot - enter its measurements"
    local choice
    read -r -p "Which robot? [1]: " choice
    if [ "${choice:-1}" != 2 ]; then
        set_values "# Worx Landroid M600 Plus: the defaults in omnimow.env"
        echo "Using the Worx Landroid defaults. Change them later: sudo bash $SCRIPT_DIR/configure_robot.sh"
        return
    fi

    # Filled in by ask
    local track="" margin="" cut="" cam_z="" cam_x="" cam_pitch="" baseline="" upside="" \
          vesc_left="" vesc_right="" erpm=""
    echo ""
    echo "Measure in centimetres from the middle between the two drive wheels (where the robot"
    echo "turns). Enter keeps the value in brackets."
    ask track "Track width: centre of the left wheel to centre of the right wheel" \
        "$(calc "$OMNIMOW_TRACK_WIDTH * 100")"
    ask front "Body length in front of the drive wheel axle (incl. bumper)" "$front"
    ask rear "Body length behind the drive wheel axle" "$rear"
    ask width "Body width at the widest point" "$width"
    ask cut "Cutting width of the blade" "$(calc "$OMNIMOW_CUT_WIDTH * 100")"
    ask margin "Extra distance Nav2 keeps from obstacles (more when the blade is at the edge)" 12
    echo "Camera (the stereo pair):"
    ask cam_z "Height of the cameras above the ground" "$(calc "$OMNIMOW_CAMERA_HEIGHT_Z * 100")"
    ask cam_x "Distance of the cameras in front of the drive wheel axle" "$(calc "$OMNIMOW_CAMERA_OFFSET_X * 100")"
    ask cam_pitch "Camera tilt downwards in degrees (0 = straight ahead)" \
        "$(calc "math.degrees($OMNIMOW_CAMERA_PITCH_Y)")"
    ask baseline "Distance between the two camera lenses in mm" "$(calc "$OMNIMOW_CAMERA_BASELINE * 1000")"
    ask upside "Camera pair mounted upside down (true/false)" "$OMNIMOW_CAMERA_UPSIDE_DOWN"
    echo "Motor controllers:"
    ask vesc_left "CAN ID of the left drive VESC" "$OMNIMOW_VESC_LEFT_ID"
    ask vesc_right "CAN ID of the right drive VESC" "$OMNIMOW_VESC_RIGHT_ID"
    ask erpm "Blade speed in ERPM (blade RPM x motor pole pairs)" "$OMNIMOW_CUTTER_ERPM"

    # Derived: the outline for Nav2, the circle around it (turning on the spot), the inflation
    # beyond it and the distance from the lawn edge to the centre when the lawn is mapped
    local f r w radius
    f=$(calc "$front / 100"); r=$(calc "$rear / 100"); w=$(calc "$width / 200")
    radius=$(calc "math.hypot(max($f, $r), $w)")
    set_values \
        "# Own robot (configure_robot.sh)" \
        "OMNIMOW_TRACK_WIDTH=$(calc "$track / 100")" \
        "OMNIMOW_FOOTPRINT=\"[[$f, $w], [$f, -$w], [-$r, -$w], [-$r, $w]]\"" \
        "OMNIMOW_ROBOT_RADIUS=$radius" \
        "OMNIMOW_NAV2_INFLATION_RADIUS=$(calc "$radius + $margin / 100")" \
        "OMNIMOW_EDGE_DISTANCE=$(calc "$w + 0.08")" \
        "OMNIMOW_CUT_WIDTH=$(calc "$cut / 100")" \
        "OMNIMOW_CAMERA_HEIGHT_Z=$(calc "$cam_z / 100")" \
        "OMNIMOW_CAMERA_OFFSET_X=$(calc "$cam_x / 100")" \
        "OMNIMOW_CAMERA_PITCH_Y=$(calc "math.radians($cam_pitch)")" \
        "OMNIMOW_CAMERA_BASELINE=$(calc "$baseline / 1000")" \
        "OMNIMOW_CAMERA_UPSIDE_DOWN=$upside" \
        "OMNIMOW_VESC_LEFT_ID=$vesc_left" \
        "OMNIMOW_VESC_RIGHT_ID=$vesc_right" \
        "OMNIMOW_CUTTER_ERPM=$erpm"
    echo ""
    echo "Saved in $ROBOT_ENV:"
    grep -v '^#' "$ROBOT_ENV" | sed 's/^/  /'
    echo "Other settings from omnimow.env can be added there too (copy the line and change it)."
    echo "Flash the drive ESP32 again if the track width or VESC IDs changed (platformio.ini)."
}

case "${1:-}" in
    "")        wizard ;;
    --set)     shift; [ $# -gt 0 ] || { echo "Usage: $0 --set KEY=VALUE ..."; exit 1; }
               set_values "$@"; echo "Saved in $ROBOT_ENV - run host_setup.sh to apply" ;;
    --show)    cat "$ROBOT_ENV" 2>/dev/null || echo "No $ROBOT_ENV yet - omnimow.env's defaults are used" ;;
    --migrate) migrate ;;
    *)         sed -n '2,21p' "$0"; exit 1 ;;
esac
