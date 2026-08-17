# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

# Initial programming is SWD (the 4-pin U5 header) — that is the one and only
# time a probe is needed, because it is what installs MCUboot. Afterwards the
# running application carries an mcumgr SMP endpoint on its second USB CDC, so
# updates go over the USB cable (tools/flash_usb.command).
#
# Note that on this PCB the probe also SUPPLIES the board: the 3.3 V rail comes
# from an LDO whose enable pin is held up only by the power button or by the
# soft latch, so with neither asserted the SWD header's own 3.3 V is the only
# thing keeping the chip alive. That is why a bare board with no battery can be
# flashed at all.
board_runner_args(jlink "--device=nrf52840_xxaa" "--speed=4000")
board_runner_args(pyocd "--target=nrf52840" "--frequency=1000000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
