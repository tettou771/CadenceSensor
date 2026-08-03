#!/usr/bin/env python3
"""Confirm the running image so MCUboot stops treating it as a test.

flash_usb.py leaves what it uploads marked for TEST, not confirmed. MCUboot
runs a test image once and reverts to the previous one on the next boot unless
something confirms it. That is a quiet trap: the board keeps working for the
rest of the session, then silently comes back on older firmware after any power
cycle — and a cadence sensor gets power-cycled every time it goes out on a bike,
so an unconfirmed image is one ride away from disappearing.

Run this after a flash you intend to keep. flash_usb.command already does.

Usage:
  confirm_usb.py [<smp-tty>]      default: auto-detect among /dev/cu.usbmodem*
"""
import asyncio
import glob
import sys

from smpclient import SMPClient
from smpclient.transport.serial import SMPSerialTransport
from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite


async def _confirm(port: str) -> bool:
    client = SMPClient(SMPSerialTransport(), port)
    await client.connect()
    try:
        state = await client.request(ImageStatesRead())
        images = getattr(state, "images", None)
        if not images:
            print(f"  {port}: no image state returned")
            return False
        for img in images:
            slot = getattr(img, "slot", None)
            active = getattr(img, "active", False)
            confirmed = getattr(img, "confirmed", False)
            print(f"  slot {slot}: active={active} confirmed={confirmed} "
                  f"pending={getattr(img, 'pending', False)}")
        running = next((i for i in images if getattr(i, "active", False)), None)
        if running is None:
            print("  no active image found")
            return False
        if getattr(running, "confirmed", False):
            print("  already confirmed — nothing to do")
            return True
        await client.request(ImageStatesWrite(hash=running.hash, confirm=True))
        after = await client.request(ImageStatesRead())
        ok = any(getattr(i, "active", False) and getattr(i, "confirmed", False)
                 for i in getattr(after, "images", []))
        print("  confirmed" if ok else "  confirm did NOT take")
        return ok
    finally:
        await client.disconnect()


async def main() -> int:
    ports = [sys.argv[1]] if len(sys.argv) > 1 else sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        print("no usbmodem ports found")
        return 1
    for port in ports:
        try:
            print(f"trying {port}")
            if await _confirm(port):
                return 0
        except Exception as e:                      # wrong CDC endpoint, etc.
            print(f"  {port}: {e}")
    print("could not confirm on any port")
    return 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
