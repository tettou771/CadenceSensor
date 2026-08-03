#!/usr/bin/env python3
"""Read the cadence sensor over BLE the way a head unit does.

Subscribes to CSC Measurement (0x2A5B) and does exactly the arithmetic a Garmin
or Wahoo does — differentiate the cumulative revolution count against the 1/1024 s
event timestamps — so if the number here is wrong, it is wrong on a head unit too.

This exists because "the console says 90 rpm" only proves the detector works. It
does not prove the notification carries the right bytes, that the timestamps
survive the 16-bit wrap, or that the head unit's own subtraction lands where we
think it does. Those are separate things to get wrong.

Usage:
  cadence_ble.py [<name-or-address>]     default: first CSC sensor found

Requires: pip install bleak
"""
import asyncio
import struct
import sys

from bleak import BleakClient, BleakScanner

CSC_SERVICE = "00001816-0000-1000-8000-00805f9b34fb"
CSC_MEASUREMENT = "00002a5b-0000-1000-8000-00805f9b34fb"
CSC_FEATURE = "00002a5c-0000-1000-8000-00805f9b34fb"
SENSOR_LOCATION = "00002a5d-0000-1000-8000-00805f9b34fb"
BATTERY_LEVEL = "00002a19-0000-1000-8000-00805f9b34fb"

LOCATIONS = {
    0: "other", 1: "top of shoe", 2: "in shoe", 3: "hip", 4: "front wheel",
    5: "left crank", 6: "right crank", 7: "left pedal", 8: "right pedal",
    9: "front hub", 10: "rear dropout", 11: "chainstay", 12: "rear wheel",
    13: "rear hub", 14: "chest", 15: "spider", 16: "chain ring",
}


class Decoder:
    """Differentiate the CSC crank fields into RPM, wraps and all."""

    def __init__(self):
        self.prev_revs = None
        self.prev_time = None

    def feed(self, data: bytes):
        flags = data[0]
        off = 1
        if flags & 0x01:            # wheel data present — this sensor sends none
            off += 6
        if not (flags & 0x02):
            return None, "no crank data in measurement"

        revs, event = struct.unpack_from("<HH", data, off)

        if self.prev_revs is None:
            self.prev_revs, self.prev_time = revs, event
            return None, f"revs={revs} t={event}  (first sample)"

        # Both fields are uint16 and both wrap. Masking the difference is the
        # whole trick, and it is why the firmware may let its counter roll over.
        drev = (revs - self.prev_revs) & 0xFFFF
        dt = (event - self.prev_time) & 0xFFFF
        self.prev_revs, self.prev_time = revs, event

        if drev == 0:
            return 0.0, f"revs={revs} t={event}  dt={dt:5d}  rpm=  0.0 (stopped)"
        if dt == 0:
            return None, f"revs={revs} t={event}  dt=0 — two events, same timestamp"

        rpm = drev * 60.0 * 1024.0 / dt
        return rpm, (f"revs={revs} t={event}  drev={drev} dt={dt:5d}  "
                     f"rpm={rpm:6.1f}")


async def main() -> int:
    want = sys.argv[1] if len(sys.argv) > 1 else None

    print("scanning for a CSC sensor...")
    devices = await BleakScanner.discover(timeout=8.0, return_adv=True)

    target = None
    for dev, adv in devices.values():
        if want:
            if want.lower() not in (dev.address.lower(), (dev.name or "").lower()):
                continue
        elif CSC_SERVICE not in [u.lower() for u in adv.service_uuids]:
            continue
        target = dev
        print(f"found {dev.name or '?'} [{dev.address}] rssi={adv.rssi}")
        break

    if target is None:
        print("no CSC sensor found (is the crank moving? it only advertises "
              "when it has recently seen motion)", file=sys.stderr)
        return 1

    async with BleakClient(target) as client:
        for uuid, label, fmt in (
            (CSC_FEATURE, "feature", lambda b: f"0x{struct.unpack('<H', b)[0]:04x}"),
            (SENSOR_LOCATION, "location",
             lambda b: f"{b[0]} ({LOCATIONS.get(b[0], '?')})"),
            (BATTERY_LEVEL, "battery", lambda b: f"{b[0]} %"),
        ):
            try:
                print(f"{label:9s}: {fmt(await client.read_gatt_char(uuid))}")
            except Exception as e:
                print(f"{label:9s}: unavailable ({e})")

        decoder = Decoder()

        def on_notify(_, data: bytearray):
            _, line = decoder.feed(bytes(data))
            print(line)

        await client.start_notify(CSC_MEASUREMENT, on_notify)
        print("\nsubscribed — pedal. ctrl-C to stop.\n")
        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        finally:
            await client.stop_notify(CSC_MEASUREMENT)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        pass
