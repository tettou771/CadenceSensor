#!/usr/bin/env bash
#
# Create an NCS west workspace for the cadence sensor firmware.
#
# This downloads the nRF Connect SDK (~GB, several minutes). The Zephyr SDK
# toolchain ($ZEPHYR_SDK_INSTALL_DIR) is shared and not re-installed here.
#
# This project needs the same NCS revision as FindMyTag, JAVELYTICS and
# RideFormTracker, so if you already have one of those workspaces, skip this
# and just point build.env's NCS_ROOT at it.
#
set -euo pipefail
cd "$(dirname "$0")"

NCS_ROOT="${1:-${NCS_ROOT:-$HOME/ncs/eggdrop}}"
NCS_REV="${NCS_REV:-v3.3.0-preview3-branch}"

echo "==> NCS workspace : $NCS_ROOT"
echo "==> sdk-nrf rev   : $NCS_REV"
echo "(this downloads several GB; ctrl-C to abort)"
echo

mkdir -p "$NCS_ROOT"

if [ ! -d "$NCS_ROOT/ncs-venv" ]; then
	python3 -m venv "$NCS_ROOT/ncs-venv"
fi
# shellcheck disable=SC1091
source "$NCS_ROOT/ncs-venv/bin/activate"
pip install --upgrade pip
# smpclient is what tools/flash_usb.py uses; pyserial is the console read-back;
# bleak is the host-side BLE test tool. pyocd is the one-time SWD flash.
pip install west pyocd smpclient pyserial bleak

cd "$NCS_ROOT"
if [ ! -d "$NCS_ROOT/.west" ]; then
	west init -m https://github.com/nrfconnect/sdk-nrf.git --mr "$NCS_REV"
fi
west update
west zephyr-export
pip install -r zephyr/scripts/requirements.txt || true

echo
echo "Done. Point firmware/build.env at it:"
echo "  NCS_ROOT=\"$NCS_ROOT\""
echo "Then:  tools/flash.command --build"
