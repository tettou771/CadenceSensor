#!/usr/bin/env python3
"""Flash the cadence sensor app over USB via mcumgr SMP — no debug probe needed.

Uploads the MCUboot-signed image to the secondary slot, marks it for test, and
resets; MCUboot swaps it in on boot. Uses upgrade=False so MCUboot's version
check is skipped (dev rebuilds share the same image version). The SMP endpoint
is the 2nd USB CDC-ACM (cdc_acm_uart1); this tries each usbmodem tty.

Usage:
  flash_usb.py [<signed.bin>] [<smp-tty>]
Defaults:
  <signed.bin> = build/firmware/zephyr/zephyr.signed.bin
  <smp-tty>    = auto-detect among /dev/cu.usbmodem*

Requires: pip install smpclient   (already in the ncs venv)
"""
import asyncio
import glob
import os
import re
import subprocess
import sys

from smpclient import SMPClient
from smpclient.transport.serial import SMPSerialTransport
from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite
from smpclient.requests.os_management import ResetWrite

FW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BIN = os.path.join(FW, "build", "firmware", "zephyr", "zephyr.signed.bin")

# Must match CONFIG_USB_DEVICE_PID in prj.conf.
OUR_PID = 0x7695
OUR_PRODUCT = "Cadence Sensor"


def usb_devices():
    """[(product, pid)] for attached USB devices. macOS only; [] elsewhere."""
    if sys.platform != "darwin":
        return []
    try:
        out = subprocess.run(
            ["ioreg", "-p", "IOService", "-c", "IOUSBHostDevice", "-r", "-w0"],
            capture_output=True, text=True, timeout=10,
        ).stdout
    except Exception:  # noqa: BLE001 - a diagnostic must never be the failure
        return []

    # ioreg emits the properties of one device before the next "+-o" header,
    # but in NO fixed order among themselves — idProduct commonly comes before
    # USB Product Name. So accumulate a whole block and flush it at the
    # boundary rather than pairing fields as they arrive.
    found = []
    product, pid = None, None

    def flush():
        if pid is not None:
            found.append((product or "?", pid))

    for line in out.splitlines():
        if re.match(r"^\s*\+-o ", line):
            flush()
            product, pid = None, None
            continue
        m = re.search(r'"USB Product Name" = "([^"]*)"', line)
        if m:
            product = m.group(1)
        m = re.search(r'"idProduct" = (\d+)', line)
        if m:
            pid = int(m.group(1))
    flush()

    return found


def explain_failure() -> None:
    """Say why SMP did not answer, which is almost never a transport problem.

    The raw error from a failed attempt is a timeout on an SMP request, which
    is true and useless: it says the device did not reply, not that this device
    was never going to reply. The overwhelmingly common cause is running this
    against a board that does not have this firmware on it yet — the SMP
    endpoint lives in OUR application, so nothing else on earth answers here.
    """
    devices = usb_devices()

    print("\n--- why this probably failed ---", file=sys.stderr)

    if devices:
        print("USB devices attached:", file=sys.stderr)
        for product, pid in devices:
            ours = " <- this firmware" if pid == OUR_PID else ""
            print(f"    {product!r}  PID 0x{pid:04X}{ours}", file=sys.stderr)

        if not any(pid == OUR_PID for _, pid in devices):
            print(
                f"\nNothing here identifies as {OUR_PRODUCT!r} (PID 0x{OUR_PID:04X}),"
                "\nso this board is not running this firmware yet — and the SMP"
                "\nendpoint this script talks to only exists inside it."
                "\n\nFlash it once by one of:"
                "\n    tools/flash.command         SWD, needs a CMSIS-DAP probe"
                "\n    tools/recover_usb.command   via the MCUboot already on the"
                "\n                                board, no probe needed",
                file=sys.stderr)
            return

    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if len(ports) < 2:
        print(
            f"\nOnly {len(ports)} usbmodem port(s). This firmware exposes TWO"
            "\n(console + SMP); one port usually means a different firmware is"
            "\nrunning. See tools/recover_usb.command.",
            file=sys.stderr)
    else:
        print(
            "\nThe firmware looks right, so this is more likely transport:"
            "\ntry unplugging and replugging, and make sure nothing else"
            "\n(a serial monitor) is holding the port open.",
            file=sys.stderr)


async def _flash(port: str, image: bytes) -> None:
    client = SMPClient(SMPSerialTransport(), port)
    await client.connect()
    try:
        # A still-in-test running image keeps the secondary slot reserved for
        # revert (upload fails NO_FREE_SLOT) — confirm slot 0 first if needed.
        states = await client.request(ImageStatesRead())
        for img in states.images:
            if getattr(img, "slot", None) == 0 and not getattr(img, "confirmed", True):
                print("  confirming the running image to free the secondary slot")
                await client.request(ImageStatesWrite(hash=img.hash, confirm=True))
                break

        # upgrade=False -> skip MCUboot's version/downgrade check for dev rebuilds
        async for off in client.upload(image, slot=0, upgrade=False):
            print(f"\r  {off}/{len(image)} B", end="", flush=True)
        print("\n  upload complete; marking the secondary image for test")

        states = await client.request(ImageStatesRead())
        secondary = None
        for img in states.images:
            if getattr(img, "slot", None) == 1:
                secondary = img
                break
        if secondary is None:
            raise RuntimeError(f"no secondary-slot image in states: {states}")

        await client.request(ImageStatesWrite(hash=secondary.hash, confirm=False))
        print("  test flag set — resetting to swap in the new image")
        await client.request(ResetWrite())
    finally:
        await client.disconnect()


def main() -> int:
    fw_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BIN
    if not os.path.isfile(fw_path):
        print(f"!!! no image at {fw_path} (build first)", file=sys.stderr)
        return 1
    with open(fw_path, "rb") as f:
        image = f.read()

    ttys = [sys.argv[2]] if len(sys.argv) > 2 else sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ttys:
        print("!!! no /dev/cu.usbmodem* — is the board's USB plugged in?", file=sys.stderr)
        return 1

    print(f"flashing {len(image)} B, trying SMP on: {', '.join(ttys)}")
    for port in ttys:
        try:
            asyncio.run(_flash(port, image))
            print(f"done via {port}")
            return 0
        except Exception as e:  # noqa: BLE001 - report and try the next tty
            print(f"  {port}: {type(e).__name__}: {e}")
    print("!!! SMP flash failed on all ttys", file=sys.stderr)
    explain_failure()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
