#!/usr/bin/env python3
"""Log the sensor's battery reading to a CSV, one sample at a time, for hours.

Written to answer "is this thing actually charging?", which the firmware cannot
answer on its own: the MCP73831's STAT output drives the yellow LED and nothing
else, so no GPIO can see it. What the firmware CAN see is the pack voltage, and
a charger either moves that or it does not.

Also useful later for the opposite measurement - leave it running on battery and
the same CSV is a discharge curve, which is the only honest way to check the
model in tools/power_budget.py against a real board.

Deliberately hard to kill. It runs unattended overnight, so a board that reboots
(a stray 'r', a brownout) or a USB port that comes and goes must not end the run
- both are logged as gaps and sampling continues.

Usage:
  battery_log.py                        # auto-detect port, 60 s, 12 h
  battery_log.py --interval 30 --hours 2
  battery_log.py --port /dev/cu.usbmodem2101 --out /tmp/batt.csv
"""
import argparse
import csv
import glob
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

PROMPT = b"s\n"
MARKER = "cadence sensor"


def candidate_ports():
    return sorted(glob.glob("/dev/cu.usbmodem*"))


def read_status(port, timeout=2.0):
    """Ask one port for a status block. Returns the text, or None."""
    with serial.Serial(port, 115200, timeout=0.5) as s:
        s.reset_input_buffer()
        s.write(PROMPT)
        s.flush()

        deadline = time.time() + timeout
        buf = ""
        while time.time() < deadline:
            chunk = s.read(1024).decode(errors="replace")
            if chunk:
                buf += chunk
                if "uptime" in buf:
                    break
        return buf if MARKER in buf else None


def find_port(preferred=None):
    """The console is one of two CDC endpoints and which one is not fixed, so
    ask each in turn rather than assuming. The SMP endpoint never answers."""
    for port in ([preferred] if preferred else []) + candidate_ports():
        if not port:
            continue
        try:
            if read_status(port):
                return port
        except (OSError, serial.SerialException):
            continue
    return None


def field(text, name, pattern):
    import re

    m = re.search(pattern, text)
    return m.group(1) if m else ""


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port")
    p.add_argument("--interval", type=float, default=60.0)
    p.add_argument("--hours", type=float, default=12.0)
    p.add_argument("--out", default="battery_log.csv")
    a = p.parse_args()

    port = find_port(a.port)
    if not port:
        sys.exit(f"no cadence sensor console found on {candidate_ports()}")

    print(f"logging {port} -> {a.out} every {a.interval:.0f}s "
          f"for {a.hours:.1f}h", flush=True)

    end = time.time() + a.hours * 3600
    started = time.time()
    prev_mv = None

    with open(a.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["wallclock", "elapsed_s", "mv", "percent",
                    "board_uptime_s", "delta_mv", "note"])
        fh.flush()

        while time.time() < end:
            note = ""
            mv = pct = up = ""
            try:
                text = read_status(port)
                if text is None:
                    # Port answered but not with a status block, or the board
                    # re-enumerated under a different name after a reboot.
                    note = "no response"
                    found = find_port(a.port)
                    if found and found != port:
                        note = f"port moved {port} -> {found}"
                        port = found
                else:
                    mv = field(text, "mv", r"battery\s*:\s*\d+ %\s*\((\d+) mV")
                    pct = field(text, "pct", r"battery\s*:\s*(\d+) %")
                    up = field(text, "up", r"uptime\s*:\s*(\d+) s")
            except (OSError, serial.SerialException) as e:
                note = f"{type(e).__name__}: {e}"
                port = find_port(a.port) or port

            delta = ""
            if mv:
                if prev_mv is not None:
                    delta = int(mv) - prev_mv
                prev_mv = int(mv)

            row = [datetime.now().isoformat(timespec="seconds"),
                   round(time.time() - started), mv, pct, up, delta, note]
            w.writerow(row)
            fh.flush()
            print("  ".join(str(c) for c in row), flush=True)

            time.sleep(a.interval)

    print("done", flush=True)


if __name__ == "__main__":
    main()
