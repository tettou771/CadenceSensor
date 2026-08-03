#
# Bicycle cadence sensor firmware
# SPDX-License-Identifier: Apache-2.0
#
cmake_minimum_required(VERSION 3.20.0)

# Custom board lives in-tree under boards/nordic/cadence_rft. Also passed on
# the command line by tools/flash.command, because sysbuild resolves the board
# for the MCUboot image before this file is ever read.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cadence_sensor)

target_sources(app PRIVATE
  src/main.c
  src/cadence.c
  src/ble_csc.c
  src/lsm6dsv.c
)
target_include_directories(app PRIVATE src)
