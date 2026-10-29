#!/bin/bash
# Grant serial access to the Robotiq 2F-85 USB-RS485 adapter. Run with sudo.
#
#   sudo ./setup_permissions.sh            # match any FTDI FT230X RS485 adapter
#   sudo ./setup_permissions.sh DAATR4XA   # pin to one adapter's serial number
#
# Pinning matters only if several FTDI 0403:6015 devices share the machine and
# they must not all get the /dev/robotiq symlink.
set -e

TARGET_USER="${SUDO_USER:-$USER}"
SERIAL="$1"

# 1. permanent group membership (applies at next login)
usermod -aG dialout "$TARGET_USER"

# 2. udev rule: world-writable node + stable /dev/robotiq symlink, so the
#    port never drifts when other USB serial devices are plugged in
MATCH='SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015"'
if [ -n "$SERIAL" ]; then
    MATCH="$MATCH, ATTRS{serial}==\"$SERIAL\""
    echo "pinning udev rule to adapter serial $SERIAL"
else
    echo "writing generic udev rule for any FTDI 0403:6015 RS485 adapter"
fi
echo "$MATCH, MODE=\"0666\", SYMLINK+=\"robotiq\"" > /etc/udev/rules.d/99-robotiq.rules

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
sleep 1

echo "--- result ---"
cat /etc/udev/rules.d/99-robotiq.rules
ls -l /dev/robotiq 2>/dev/null || echo "WARNING: /dev/robotiq not created - is the adapter plugged in?"
echo "'$TARGET_USER' groups: $(id -nG "$TARGET_USER")"
