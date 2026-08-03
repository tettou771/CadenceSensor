#!/usr/bin/env bash
# Build and run the host-side cadence detector exercise.
#
#   test/run.sh        run the scenarios
#   test/run.sh -v     ...with the detector's own log lines on stderr
#
# No Zephyr, no board, no SDK — test/zephyr/ stubs just enough kernel API for
# src/cadence.c to compile against a normal host compiler.
#
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$DIR/test_cadence"

# cadence_thread() is reachable only through K_THREAD_DEFINE, which the stub
# turns into nothing — unused here, very much used on the board.
cc -std=c11 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function \
   -I "$DIR" -I "$DIR/../src" \
   -o "$OUT" "$DIR/test_cadence.c" -lm

"$OUT" "$@"
