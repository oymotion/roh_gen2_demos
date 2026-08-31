#!/bin/sh
# One-time install of udev rules for the sensor-sdk USB Bluetooth dongle (Linux).
#
# Usage:
#   sudo sensor/tools/setup_dongle_udev.sh
#
# After installation, regular users can access the dongle (one replug may be
# needed; the script tries to trigger re-enumeration automatically):
#   SENSOR_SDK_BLE_BACKEND=bumble python examples/console.py
#
# WSL note: udevadm trigger may disconnect usbipd-attached devices; if that
# happens, re-attach from the Windows side: usbipd attach --wsl --busid <id>

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RULES_SRC="$SCRIPT_DIR/50-sensor-dongle.rules"
RULES_DST="/etc/udev/rules.d/50-sensor-dongle.rules"

if [ "$(id -u)" -ne 0 ]; then
    echo "error: root required, please run: sudo $0" >&2
    exit 1
fi

if [ ! -f "$RULES_SRC" ]; then
    echo "error: rules file not found: $RULES_SRC" >&2
    exit 1
fi

install -m 644 "$RULES_SRC" "$RULES_DST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb

echo "udev rules installed to $RULES_DST"
echo "If the dongle is already plugged in but still not accessible, unplug and replug it once."
echo "Then run as a regular user: SENSOR_SDK_BLE_BACKEND=bumble python examples/console.py"
