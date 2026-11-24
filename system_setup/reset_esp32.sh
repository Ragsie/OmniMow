#!/usr/bin/env bash
# OmniMow: restart both ESP32s from the Radxa through their USB serial ports.
#
# The drive ESP32 locks its emergency stop when the heartbeat from app_backend stops (the motor
# relay stays off until it restarts). That is wanted during a software update: the mower cannot
# drive while its services restart. update_software.sh therefore reboots the mower after an update
# and leaves /opt/omnimow/esp32_reset_pending; at boot, omnimow-esp32-reset.service runs this script
# before Docker starts, so both ESP32s start fresh and only arm their watchdog again on the first
# heartbeat.
#
# The ESP32 boards' auto-reset circuit (CP2102: RTS -> EN, DTR -> IO0, as used for flashing) is
# pulsed: EN low for 0.2 s with IO0 high, so the ESP32 boots its normal firmware.
#
#   reset_esp32.sh            restart both ESP32s now (also by hand: sudo omnimow-reset-esp32)
#   reset_esp32.sh --pending  only if /opt/omnimow/esp32_reset_pending exists (at boot), then remove it
set -euo pipefail

PENDING=/opt/omnimow/esp32_reset_pending
if [ "${1:-}" = --pending ]; then
    [ -e "$PENDING" ] || exit 0
    rm -f "$PENDING"
fi

status=0
for dev in /dev/ttyUSB_esp32_drive /dev/ttyUSB_esp32_cutter; do
    # At boot udev may still be creating the links
    for _ in $(seq 30); do
        [ -e "$dev" ] && break
        sleep 0.5
    done
    if [ ! -e "$dev" ]; then
        echo "$dev not found - not restarted (ESP32 not connected, or its serial not in 99-omnimow.rules)"
        status=1
        continue
    fi
    if python3 - "$dev" << 'EOF'
import fcntl, os, struct, sys, termios, time
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
def line(bit, on):
    fcntl.ioctl(fd, termios.TIOCMBIS if on else termios.TIOCMBIC, struct.pack('I', bit))
line(termios.TIOCM_DTR, False)  # IO0 high: boot the firmware, not the bootloader
line(termios.TIOCM_RTS, True)   # EN low: hold the ESP32 in reset
time.sleep(0.2)
line(termios.TIOCM_RTS, False)  # EN high: run
os.close(fd)
EOF
    then
        echo "Restarted the ESP32 on $dev"
    else
        echo "Could not restart the ESP32 on $dev"
        status=1
    fi
done
exit $status
