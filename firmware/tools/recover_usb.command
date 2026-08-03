#!/usr/bin/env bash
#
# Flash the application through MCUboot's SERIAL RECOVERY, over USB, with no
# debug probe.
#
# This is the odd one out of the three flashing tools, and it exists for one
# specific situation: a board that already carries somebody else's MCUboot.
#
#   tools/flash.command       SWD + probe. Installs OUR MCUboot. The normal
#                             first flash, and the only way back from a dead
#                             application.
#   tools/flash_usb.command   SMP over the RUNNING APPLICATION's second CDC.
#                             The normal update path — but it needs our
#                             application already on the board to answer.
#   tools/recover_usb.command (this) SMP over the BOOTLOADER's CDC, during the
#                             DFU window it opens at power-up. Needs no probe
#                             and no cooperation from whatever application is
#                             currently installed.
#
# The RideFormTracker PCB this project borrows arrives with the RideFormTracker
# firmware, whose MCUboot was built with serial recovery enabled and — this is
# the part that makes this work — the SAME partition layout and the SAME
# "no signature" policy as ours:
#
#     mcuboot            0x00000 - 0x20000     same
#     mcuboot_primary    0x20000 - 0x88000     same
#     mcuboot_secondary  0x88000 - 0xf0000     same
#     signature          BOOT_SIGNATURE_TYPE_NONE    same
#
# So its bootloader will accept our image. What you end up with is RFT's
# MCUboot underneath our cadence application, which is a perfectly good place
# to be: our app brings its own SMP endpoint, so from then on the ordinary
# flash_usb.command works too.
#
# The catch is timing. That MCUboot waits only 1 SECOND at power-up for a DFU
# connection (CONFIG_BOOT_SERIAL_WAIT_FOR_DFU_TIMEOUT=1000), so this script
# watches for the port to appear and pounces. Hence the power-cycle prompt.
#
# SPDX-License-Identifier: Apache-2.0
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(cd "$DIR/.." && pwd)"

# shellcheck disable=SC1091
[ -f "$FW/build.env" ] && source "$FW/build.env"

MCUMGR="${MCUMGR:-$HOME/go/bin/mcumgr}"
BUILD_DIR="${BUILD_DIR:-$FW/build}"
# Sysbuild names the application image directory after the app SOURCE
# DIRECTORY (firmware/), not after the CMake project name.
BIN_FILE="${BIN_FILE:-$BUILD_DIR/firmware/zephyr/zephyr.signed.bin}"

SND="/System/Library/Sounds"
beep_start()   { afplay "$SND/Tink.aiff" & }
beep_success() { afplay "$SND/Glass.aiff" & }
beep_fail()    { afplay "$SND/Basso.aiff" & }

die() { echo "ERROR: $*" >&2; beep_fail; read -rp "Press Enter to close..."; exit 1; }

[ -x "$MCUMGR" ] || die "mcumgr not found at $MCUMGR
  install: go install github.com/apache/mynewt-mcumgr-cli/mcumgr@latest"

if [ ! -f "$BIN_FILE" ]; then
	BIN_FILE="$(find "$BUILD_DIR" -name zephyr.signed.bin 2>/dev/null | head -1)"
fi
[ -n "$BIN_FILE" ] && [ -f "$BIN_FILE" ] || die "no zephyr.signed.bin under $BUILD_DIR
  build first:  ./tools/flash.command --build   (the build works without a probe;
                it only fails at the flashing step)"

echo "=== cadence sensor — MCUboot serial recovery ==="
echo "image: $BIN_FILE"
echo
echo "This overwrites the application currently on the board."
echo

while true; do
	echo "--- Press Enter, THEN power-cycle the board (Ctrl-C to quit) ---"
	echo "    (unplug/replug USB, or flip SW1 off and on)"
	read -r

	# Ignore ports that are already there; we want the one that APPEARS.
	# The port the running application owns is not the bootloader's, and
	# talking to the wrong one is exactly the timeout this script exists to
	# avoid.
	EXISTING=$(ls /dev/cu.usbmodem* 2>/dev/null)

	echo -n "waiting for the bootloader to enumerate"
	SERIAL=""
	while [ -z "$SERIAL" ]; do
		for dev in /dev/cu.usbmodem*; do
			[ -e "$dev" ] || continue
			if ! echo "$EXISTING" | grep -qF "$dev"; then
				SERIAL="$dev"
				break
			fi
		done
		[ -z "$SERIAL" ] && { echo -n "."; sleep 0.3; }
	done
	echo " found: $SERIAL"
	sleep 0.3

	beep_start
	OPTS=(--conntype serial --connstring "$SERIAL,baud=115200")

	if ! "$MCUMGR" "${OPTS[@]}" image upload "$BIN_FILE"; then
		echo "=== upload FAILED ==="
		echo "  If this timed out, the 1 s DFU window was missed — the board"
		echo "  had already left the bootloader and started the old app."
		echo "  Press Enter and power-cycle again, a little more promptly."
		beep_fail
		echo
		continue
	fi

	# Confirm the freshly uploaded image if the bootloader parked it in
	# slot 1. Serial recovery usually writes the primary slot directly, in
	# which case there is nothing to confirm — and that is not an error.
	LIST=$("$MCUMGR" "${OPTS[@]}" image list 2>&1)
	echo "$LIST"
	HASH=$(echo "$LIST" | awk '/slot=1/{f=1} f && /hash:/{print $2; exit}')

	if [ -n "$HASH" ]; then
		"$MCUMGR" "${OPTS[@]}" image confirm "$HASH" ||
			echo "(confirm failed — resetting anyway)"
	fi

	"$MCUMGR" "${OPTS[@]}" reset

	echo
	echo "=== update OK ==="
	echo
	echo "What to expect on the next boot:"
	echo "  - TWO /dev/cu.usbmodem* ports, not one (console + SMP)."
	echo "    Still one port means the old application is still running."
	echo "  - the USB product string becomes 'Cadence Sensor' (PID 0x7695),"
	echo "    where the RideFormTracker firmware said 'RFT Tracker' (0x7692)."
	echo "  - open the LOWER-numbered port and press 's'."
	echo "  - from here on, tools/flash_usb.command works and is nicer:"
	echo "    no power-cycle, no window to hit."
	beep_success
	echo
done
