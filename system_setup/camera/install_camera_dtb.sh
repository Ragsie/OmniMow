#!/usr/bin/env bash
# Enables the OmniMow stereo cameras (2x IMX219 on CAM2 + CAM3) on a systemd-boot image whose
# device tree has no __symbols__ (e.g. rafaelwms/ubuntu-26.04.1-dragon-q6a), where overlays
# cannot be applied at boot.
#
# Builds a copy of the kernel's board .dtb with dual-imx219.dtsi.in merged in and adds a
# separate systemd-boot entry "OmniMow cameras" that boots with it. The original entry stays,
# so the board can always be started without the change from the boot menu.
#
#   install_camera_dtb.sh [KERNEL_VERSION]   build for the running (or given) kernel
#   install_camera_dtb.sh --default          also make the camera entry the default boot entry
#                                            (if it does not boot, pick the normal entry in the
#                                            boot menu and run --remove)
#   install_camera_dtb.sh --remove           remove the camera entries and the default setting
#
# Installed as /etc/kernel/postinst.d/zz-omnimow-cameras by host_setup.sh, so a kernel update
# gets its own camera entry.
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
TEMPLATE="$SRC_DIR/dual-imx219.dtsi.in"
BOARD_DTB=qcom/qcs6490-radxa-dragon-q6a.dtb
ESP=/boot/efi
ENTRIES="$ESP/loader/entries"
LOADER_CONF="$ESP/loader/loader.conf"
OUT_DIR="$ESP/omnimow"

KVER="$(uname -r)"
MAKE_DEFAULT=false
case "${1:-}" in
    --default) MAKE_DEFAULT=true ;;
    --remove)
        rm -f "$ENTRIES"/omnimow-cameras-*.conf
        find "$OUT_DIR" -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} + 2>/dev/null || true   # keeps firmware.dtb
        [ -f "$LOADER_CONF" ] && sed -i '/^default omnimow-cameras/d' "$LOADER_CONF"
        echo "Camera boot entries removed"
        exit 0 ;;
    "") ;;
    *) KVER="$1" ;;   # From /etc/kernel/postinst.d: the new kernel's version
esac

if [ ! -d "$ENTRIES" ]; then
    echo "No systemd-boot entries in $ENTRIES - cannot add a camera boot entry here"
    exit 1
fi
# Base device tree: preferably the UEFI firmware's own (what the board boots without the camera
# entry), which can differ from the kernel's .dtb in what the NPU (FastRPC) and other firmware
# services need. It can only be saved while the board runs on it, i.e. before the camera entry is
# used or after --remove and a reboot.
FW_DTB="$OUT_DIR/firmware.dtb"
if [ ! -e /proc/device-tree/ext-cam-clk-imx219 ] && [ -r /sys/firmware/fdt ]; then
    mkdir -p "$OUT_DIR"
    cp /sys/firmware/fdt "$FW_DTB.part"
    # Drop what the EFI stub put into /chosen for this boot; it sets it again on every boot
    for prop in bootargs linux,initrd-start linux,initrd-end linux,uefi-system-table \
                linux,uefi-mmap-start linux,uefi-mmap-size linux,uefi-mmap-desc-size \
                linux,uefi-mmap-desc-ver linux,elfcorehdr kaslr-seed rng-seed; do
        fdtput -d "$FW_DTB.part" /chosen "$prop" 2>/dev/null || true
    done
    mv "$FW_DTB.part" "$FW_DTB"
    echo "Saved the firmware's device tree: $FW_DTB"
fi
base=""
for f in "$FW_DTB" "/boot/dtb-$KVER/$BOARD_DTB" "/usr/lib/linux-image-$KVER/$BOARD_DTB" "/boot/dtbs/$KVER/$BOARD_DTB"; do
    [ -f "$f" ] && { base="$f"; break; }
done
if [ -z "$base" ]; then
    echo "No $BOARD_DTB for kernel $KVER"
    exit 1
fi
entry=$(grep -l "^linux.*$KVER" "$ENTRIES"/*.conf 2>/dev/null | grep -v '/omnimow-cameras-' | head -1 || true)
if [ -z "$entry" ]; then
    echo "No systemd-boot entry for kernel $KVER in $ENTRIES"
    exit 1
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
dtc -q -I dtb -O dts -o "$work/base.dts" "$base"

# Node paths for the labels Radxa's overlays use (the .dtb has no labels): found by compatible
# string and regulator name in the decompiled tree
read -r tlmm camss cci1 vreg_l10c vreg_l6b < <(python3 - "$work/base.dts" << 'EOF'
import re, sys
path, found = [], {}
for line in open(sys.argv[1]):
    s = line.strip()
    m = re.match(r'^(?:[\w,.+-]+:\s*)?([\w,.+@-]+|/)\s*\{$', s)
    if m:
        path.append(m.group(1))
        continue
    if s == '};':
        path.pop()
        continue
    node = '/' + '/'.join(path[1:])
    if s.startswith('compatible') and ('sc7280-pinctrl' in s or 'qcm6490-tlmm' in s):
        found.setdefault('tlmm', node)
    elif s.startswith('compatible') and 'sc7280-camss' in s:
        found.setdefault('camss', node)
    elif s.startswith('regulator-name') and '"vreg_l10c_0p88"' in s:
        found.setdefault('l10c', node)
    elif s.startswith('regulator-name') and '"vreg_l6b_1p2"' in s:
        found.setdefault('l6b', node)
keys = ['tlmm', 'camss', 'l10c', 'l6b']
missing = [k for k in keys if k not in found]
if missing:
    sys.exit('not found in the board device tree: ' + ', '.join(missing))
print(found['tlmm'], found['camss'], '/soc@0/cci@ac4b000', found['l10c'], found['l6b'])
EOF
)
grep -q "cci@ac4b000 {" "$work/base.dts" || { echo "No cci@ac4b000 (CCI1) in $base"; exit 1; }
echo "Board device tree: $base"
echo "  tlmm $tlmm, camss $camss, cci1 $cci1, l10c $vreg_l10c, l6b $vreg_l6b"

sed -e "s|@TLMM@|$tlmm|g" -e "s|@CAMSS@|$camss|g" -e "s|@CCI1@|$cci1|g" \
    -e "s|@VREG_L10C@|$vreg_l10c|g" -e "s|@VREG_L6B@|$vreg_l6b|g" "$TEMPLATE" > "$work/cameras.dtsi"
cat "$work/base.dts" "$work/cameras.dtsi" > "$work/merged.dts"
mkdir -p "$OUT_DIR/$KVER"
dtc -q -I dts -O dtb -o "$OUT_DIR/$KVER/qcs6490-radxa-dragon-q6a-cameras.dtb" "$work/merged.dts"

# Boot entry: the kernel's own entry plus the camera device tree
cam_entry="$ENTRIES/omnimow-cameras-$KVER.conf"
{
    echo "title      OmniMow cameras ($KVER)"
    grep -vE '^(title|devicetree|devicetree-overlay)[[:space:]]' "$entry"
    echo "devicetree /omnimow/$KVER/qcs6490-radxa-dragon-q6a-cameras.dtb"
} > "$cam_entry"
echo "Boot entry: $cam_entry (from $(basename "$entry"))"

# Entries for kernels that are gone
for e in "$ENTRIES"/omnimow-cameras-*.conf; do
    v=${e##*/omnimow-cameras-}; v=${v%.conf}
    if [ ! -d "/lib/modules/$v" ]; then
        rm -f "$e"
        rm -rf "${OUT_DIR:?}/$v"
    fi
done

if [ "$MAKE_DEFAULT" = true ]; then
    sed -i '/^default /d' "$LOADER_CONF"
    echo "default omnimow-cameras-*" >> "$LOADER_CONF"
    echo "The camera entry is now the default boot entry"
fi
