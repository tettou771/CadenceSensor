#!/usr/bin/env bash
# Cadence sensor firmware flash helper (double-clickable on macOS).
#
#   flash.command            flash the existing built binary (no rebuild)
#   flash.command --build    build first, then flash
#   flash.command --clean    wipe build/, rebuild, then flash
#
# Flashes the nRF52840 over a CMSIS-DAP / DAPLink probe with pyocd, writing
# merged.hex = MCUboot + signed app. This is needed exactly ONCE per board,
# because what it installs is MCUboot; after that use flash_usb.command, which
# needs no probe and no case opened.
#
# Env overrides (or put them in ../build.env, which is sourced if present):
#   NCS_ROOT                 nRF Connect SDK west workspace (has ncs-venv/, zephyr/)
#   ZEPHYR_SDK_INSTALL_DIR   Zephyr SDK toolchain root
#   BOARD                    Zephyr board target
#   PYOCD_TARGET             pyocd target name
#
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(cd "$DIR/.." && pwd)"

# shellcheck disable=SC1091
[ -f "$FW/build.env" ] && source "$FW/build.env"

NCS_ROOT="${NCS_ROOT:-$HOME/ncs/eggdrop}"
export ZEPHYR_SDK_INSTALL_DIR="${ZEPHYR_SDK_INSTALL_DIR:-$HOME/zephyr-sdk-0.17.0}"
BOARD="${BOARD:-cadence_rft}"
PYOCD_TARGET="${PYOCD_TARGET:-nrf52840}"

do_build=0
for arg in "$@"; do
	case "$arg" in
		--build) do_build=1 ;;
		--clean) do_build=1; rm -rf "$FW/build" ;;
		-h|--help) sed -n '2,18p' "${BASH_SOURCE[0]}"; exit 0 ;;
		*) echo "unknown arg: $arg" >&2; exit 2 ;;
	esac
done

# activate the SDK python venv + point Zephyr at the pinned tree
# shellcheck disable=SC1091
source "$NCS_ROOT/ncs-venv/bin/activate"
export ZEPHYR_BASE="$NCS_ROOT/zephyr"

HEX="$FW/build/merged.hex"

if [ "$do_build" = 1 ]; then
	echo ">>> building (board=$BOARD)"
	west build --sysbuild -b "$BOARD" -d "$FW/build" -s "$FW" \
		-- -DBOARD_ROOT="$FW"
fi

if [ ! -f "$HEX" ]; then
	echo "!!! no binary at $HEX — run with --build first" >&2
	exit 1
fi

# Pre-flight: is there ANY probe at all?
#
# Deliberately not the obvious check. Grepping `pyocd list` for "dap|cmsis"
# refuses to run against any probe whose description does not happen to contain
# those words — a J-Link included — and a false refusal is worse than a failed
# attempt. But dropping the check entirely is worse still: with nothing
# attached, `pyocd flash` does not fail, it BLOCKS, and a script that hangs
# forever tells you less than one that refuses wrongly.
#
# So the test is only for the one thing pyocd states unambiguously and without
# reference to probe type: whether it found anything.
probe_list=$(pyocd list 2>&1 || true)

if echo "$probe_list" | grep -qi "no available debug probes"; then
	echo "!!! no debug probe found. Things worth checking, in the order" >&2
	echo "!!! they usually go wrong:" >&2
	echo "!!!   - is the probe plugged into THIS Mac, not just into the board?" >&2
	echo "!!!   - is the USB cable a DATA cable? charge-only cables are the" >&2
	echo "!!!     single most common cause and look identical" >&2
	echo "!!!   - try the other end / another port; hubs without power fail here" >&2
	echo "!!! pyocd says:" >&2
	echo "$probe_list" | sed 's/^/!!!   /' >&2
	exit 1
fi

echo ">>> probe(s) found:"
echo "$probe_list" | sed 's/^/    /'

# nRF52840-QIAA-D0 locks SWD via APPROTECT after a chip erase. AHB-AP is dead
# at that point, so recovery has to go through CTRL-AP (AP#1), which stays
# reachable. A mass erase there clears the lock (and wipes UICR, which
# boards/nordic/cadence_rft/board.c repairs on the next boot).
#
# The `|| true` on each read is not decoration: under `set -o pipefail` a grep
# that matches nothing fails the whole pipeline, and `set -e` would then kill
# the script mid-diagnosis — exactly when it has something useful to say.
ctrl_ap_read() {
	pyocd commander -t "$PYOCD_TARGET" -N \
		-c "initdp; makeap 1; readap 1 $1; quit" 2>&1 |
		grep -o '0x[0-9a-f]*' | tail -1 || true
}

unlock_approtect() {
	local status
	status=$(ctrl_ap_read 0x00C)

	# Empty means CTRL-AP itself did not answer. CTRL-AP stays reachable
	# even on a locked part, so silence here is not "unlocked" — it is no
	# electrical connection to a device at all. Treating it as success
	# would send the caller off to retry a flash that cannot work.
	if [ -z "$status" ]; then
		echo "!!! CTRL-AP did not respond — no SWD connection" >&2
		return 1
	fi

	if [ "$status" != "0x00000001" ]; then
		echo ">>> device is not APPROTECT-locked (status $status)"
		return 0
	fi

	echo ">>> device is LOCKED — mass erasing via CTRL-AP"
	pyocd commander -t "$PYOCD_TARGET" -N \
		-c "initdp; makeap 1; writeap 1 0x004 1; quit" 2>/dev/null || true

	for _ in $(seq 1 10); do
		sleep 1
		status=$(ctrl_ap_read 0x008)
		if [ "$status" = "0x00000000" ]; then
			echo ">>> unlocked (UICR wiped; board.c restores REGOUT0 on first boot)"
			return 0
		fi
	done

	echo "!!! CTRL-AP mass erase timed out" >&2
	return 1
}

# Try the straightforward flash first. An unlocked board takes this path and
# never pays for the CTRL-AP dance.
echo ">>> flashing $HEX"
if pyocd flash -t "$PYOCD_TARGET" "$HEX" --frequency 1000000; then
	echo ">>> done — board resets and runs the new firmware"
	echo ">>> from here on, use flash_usb.command (no probe needed)"
	exit 0
fi

echo
echo ">>> flash failed — checking whether the device is APPROTECT-locked"

if ! unlock_approtect; then
	echo
	echo "!!! could not reach the device over SWD at all." >&2
	echo "!!! Things worth checking, in the order they usually go wrong:" >&2
	echo "!!!   - is the probe plugged into THIS Mac (not just into the board)?" >&2
	echo "!!!   - is the USB cable a data cable? charge-only cables are the" >&2
	echo "!!!     single most common cause and look identical" >&2
	echo "!!!   - SWDIO to U5 pad 3, SWCLK to pad 4, GND to pad 2" >&2
	echo "!!!   - the board needs its own power; the SWD header does not" >&2
	echo "!!!     supply any" >&2
	echo "!!! What the tools see right now:" >&2
	pyocd list 2>&1 | sed 's/^/!!!   /' >&2
	exit 1
fi

echo
echo ">>> flashing after unlock"
for attempt in 1 2 3 4 5; do
	if pyocd flash -t "$PYOCD_TARGET" "$HEX" --frequency 1000000; then
		echo ">>> done — board resets and runs the new firmware"
		echo ">>> the first boot runs TWICE: board.c rewrites the UICR the"
		echo ">>> mass erase wiped, then reboots. That is expected."
		echo ">>> from here on, use flash_usb.command (no probe needed)"
		exit 0
	fi
	echo ">>> attempt $attempt failed, retrying"
	sleep 1
done

echo "!!! flash failed after 5 attempts — reseat the probe and retry" >&2
exit 1
