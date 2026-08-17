#!/usr/bin/env bash
# Cadence sensor USB flash helper (double-clickable on macOS).
#
#   flash_usb.command            flash the existing built binary, then confirm
#   flash_usb.command --build    build first, then flash and confirm
#
# Flashes over USB via mcumgr SMP — no debug probe needed, and no power cycle
# to time either: the SMP endpoint belongs to the RUNNING application, on its
# second CDC-ACM. The board must already have MCUboot on it (a virgin board
# needs flash.command and a CMSIS-DAP probe once).
#
# Confirming is not optional here, and that is deliberate. flash_usb.py leaves
# what it uploads marked for TEST; MCUboot runs a test image once and reverts on
# the next boot. The board keeps working for the rest of the session and then
# quietly comes back on older firmware after any power cycle — which for a
# cadence sensor means the next ride. So this script always confirms, then reads
# the result back off the device so the outcome is visible rather than assumed.
#
# Env overrides (or put them in ../build.env, which is sourced if present):
#   NCS_ROOT   nRF Connect SDK west workspace (has ncs-venv/, zephyr/)
#   BOARD      Zephyr board target
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

do_build=0
for arg in "$@"; do
	case "$arg" in
		--build) do_build=1 ;;
		-h|--help) sed -n '2,22p' "${BASH_SOURCE[0]}"; exit 0 ;;
		*) echo "unknown arg: $arg" >&2; exit 2 ;;
	esac
done

# shellcheck disable=SC1091
source "$NCS_ROOT/ncs-venv/bin/activate"
export ZEPHYR_BASE="$NCS_ROOT/zephyr"

if [ "$do_build" = 1 ]; then
	echo ">>> building (board=$BOARD)"
	west build --sysbuild -b "$BOARD" -d "$FW/build" -s "$FW" \
		-- -DBOARD_ROOT="$FW"
fi

# Sysbuild names the application image directory after the app SOURCE
# DIRECTORY (firmware/), not after the CMake project name.
BIN="$FW/build/firmware/zephyr/zephyr.signed.bin"
if [ ! -f "$BIN" ]; then
	echo "!!! no binary at $BIN — run with --build first" >&2
	exit 1
fi

# Same guard as flash.command, and it matters more here: this path CONFIRMS the
# image, so an image for the wrong board does not merely misbehave, it is made
# permanent and survives the power cycle that would otherwise back it out.
built_board=$(grep -h '^CONFIG_BOARD=' "$FW"/build/*/zephyr/.config 2>/dev/null |
	sed 's/^CONFIG_BOARD="\(.*\)"$/\1/' | sort -u || true)

if [ -n "$built_board" ] && [ "$built_board" != "$BOARD" ]; then
	echo "!!! build/ holds an image built for: $(echo "$built_board" | tr '\n' ' ')" >&2
	echo "!!! but BOARD is '$BOARD' (from build.env)." >&2
	echo "!!! Refusing to flash — re-run with --build." >&2
	exit 1
fi

echo ">>> flashing over USB"
python3 "$DIR/flash_usb.py"

# MCUboot swaps the image in on the reset above, which takes ~20 s, and the CDC
# ports go away and come back. Waiting for "ports exist" is not enough: the OLD
# ports are still enumerated for a moment after the reset request, so that test
# passes instantly and the confirm below then finds nothing. Wait for them to
# GO first, then to return.
echo ">>> waiting for the board to reset"
for _ in $(seq 30); do
	compgen -G "/dev/cu.usbmodem*" > /dev/null || break
	sleep 1
done
echo ">>> waiting for the swap to finish and USB to come back"
for _ in $(seq 90); do
	if compgen -G "/dev/cu.usbmodem*" > /dev/null; then break; fi
	sleep 1
done
sleep 3   # let both CDC endpoints enumerate and the app finish booting

# Retry regardless: the port can be present a beat before SMP answers.
echo ">>> confirming the image (so it survives a power cycle)"
confirmed=0
for _ in $(seq 6); do
	if python3 "$DIR/confirm_usb.py"; then confirmed=1; break; fi
	sleep 3
done
if [ "$confirmed" != 1 ]; then
	echo "!!! COULD NOT CONFIRM — the board will revert to the previous"
	echo "!!! firmware on its next power cycle. Re-run before using it."
	exit 1
fi

echo
echo ">>> reading the result back off the board"
python3 - <<'PY'
import glob, sys, time
try:
    import serial
except ImportError:
    sys.exit("pyserial not available; skipping verification")

# The console is cdc_acm_uart0, the SMP endpoint is uart1. macOS enumerates
# them in order, so the lower-numbered tty is the console.
ports = sorted(glob.glob('/dev/cu.usbmodem*'))
if not ports:
    sys.exit("no console port found")
s = serial.Serial(ports[0], 115200, timeout=0.3)
time.sleep(1.5)
s.reset_input_buffer()
s.write(b's\n')

# The lines worth a human's attention: anything here reading wrong means the
# board is not configured the way the firmware thinks it is.
KEYS = ('name', 'BLE addr', 'IMU', 'sampler', 'plane')
end = time.time() + 6
for line in iter(lambda: s.readline().decode('utf-8', 'replace').rstrip(), None):
    if line and any(line.startswith(k) for k in KEYS):
        print('   ', line)
    if time.time() > end:
        break
s.close()
PY

echo
echo ">>> done. Check above that:"
echo "      IMU reads ok (not ABSENT)"
echo "      BLE addr is stable across reflashes (it comes from FICR)"
echo "      spin the crank and 'plane' settles on two axes, amp near 1000 mg"
