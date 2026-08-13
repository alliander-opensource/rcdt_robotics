#!/usr/bin/env python
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


"""ublox_serial.py - List and configure u-blox GNSS device serial strings.

Usage:
    python3 ublox_serial.py list
    python3 ublox_serial.py set <bus-port> <string>
    python3 ublox_serial.py set-persistent <bus-port> <string>

Examples:
    python3 ublox_serial.py list
    python3 ublox_serial.py set 1-2.1 ROVER001
    python3 ublox_serial.py set-persistent 1-2.2 BASE001
"""

import struct
import subprocess
import sys
from pathlib import Path
from typing import Literal

UBLOX_VENDOR_ID = "1546"
UBLOX_PRODUCT_IDS = {
    "01ab": "ZED-X20P / ZED-F20P",
    "01a9": "ZED-F9P / ZED-F9R",
}
UBXTOOL = "ubxtool"
UBXTOOL_PROTO = "50"
UBXTOOL_BAUD = "115200"
MAX_SERIAL_STR_LENGTH = 8


def get_ublox_devices() -> list[dict]:
    """Gets a list of U-blox devices currently attached to this device by checking idVendor and idProduct.

    Returns:
        list[dict]: list containing U-Blox devices and their port, product type, tty device, and iSerial number.
    """
    devices = []
    usb_root = Path("/sys/bus/usb/devices")

    for dev_dir in sorted(usb_root.iterdir()):
        vendor_file = dev_dir / "idVendor"
        product_file = dev_dir / "idProduct"
        if not vendor_file.exists():
            continue
        if vendor_file.read_text().strip() != UBLOX_VENDOR_ID:
            continue
        product_id = product_file.read_text().strip()
        if product_id not in UBLOX_PRODUCT_IDS:
            continue

        tty = _find_tty(dev_dir)
        serial = _find_iserial(dev_dir)

        devices.append(
            {
                "port_path": dev_dir.name,
                "product": UBLOX_PRODUCT_IDS[product_id],
                "tty": tty,
                "iserial": serial,
            }
        )

    return devices


def _find_tty(dev_dir: Path) -> str | None:
    for uevent in dev_dir.rglob("tty*/uevent"):
        for line in uevent.read_text().splitlines():
            if line.startswith("DEVNAME="):
                return "/dev/" + line.split("=", 1)[1].strip()

    return None


def _find_iserial(dev_dir: Path) -> str:
    serial_file = dev_dir / "serial"
    if serial_file.exists():
        return serial_file.read_text().strip()
    return "(none)"


def cmd_list() -> None:
    """Lists the available U-Blox devices in a pretty format."""
    devices = get_ublox_devices()
    if not devices:
        print("No u-blox devices found.")
        return

    print(f"{'Port path':<12} {'iSerial':<12} {'TTY':<16} Product")
    print("-" * 64)
    for d in devices:
        tty = d["tty"] or "(no tty)"
        print(f"{d['port_path']:<12} {d['iserial']:<12} {tty:<16} {d['product']}")


def cmd_set(
    port_path: str, serial_string: str, memory_layer: Literal[1, 7] = 1
) -> None:
    """Updates a specific device's CFG-USB-SERIAL_NO_STR0 to a given value. This makes iSerial access for ublox_dgnss possible.

    Args:
        port_path (str): USB device port path, e.g. 1-2.1.
        serial_string (str): Serial string to update CFG-USB-SERIAL_NO_STR0 to.
        memory_layer (Literal[1, 7]): Whether to flash to RAM only (1), or to persistent memory (7).
    """
    if len(serial_string) > MAX_SERIAL_STR_LENGTH:
        print(
            f"Error: serial string '{serial_string}' is {len(serial_string)} characters. Maximum is 8."
        )
        sys.exit(1)

    # Pad to exactly 8 bytes and pack little-endian into a uint64
    padded = serial_string.encode("ascii").ljust(8, b"\x00")
    packed_value = struct.unpack("<Q", padded)[0]

    devices = get_ublox_devices()
    match = next((d for d in devices if d["port_path"] == port_path), None)
    if match is None:
        print(f"Error: no u-blox device found at port path '{port_path}'.")
        print("Run 'list' to see available devices.")
        sys.exit(1)

    tty = match["tty"]
    if tty is None:
        print(f"Error: device at {port_path} has no tty — is cdc_acm bound?")
        sys.exit(1)

    print(f"Setting serial string '{serial_string}' on {port_path} ({tty})")
    print(f"  Packed value: {packed_value} (0x{packed_value:016x})")

    result = subprocess.run(
        [
            UBXTOOL,
            "-P",
            UBXTOOL_PROTO,
            "-f",
            tty,
            "-s",
            UBXTOOL_BAUD,
            "-z",
            f"CFG-USB-SERIAL_NO_STR0,{packed_value},{memory_layer}",
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    output = result.stdout + result.stderr
    if "ACK-ACK" in output:
        print("  ACK received — success.")
        print(
            "  The operation will not have an effect until you plug the USB device in again."
        )
    elif "NAK" in output:
        print("  NAK received — command rejected by device.")
    else:
        print("  No ACK/NAK found in output:")
        print(output)


def main() -> None:
    """Process command-line arguments and call the corresponding function."""
    min_sys_args = 2
    len_set_sys_args = 4

    if len(sys.argv) < min_sys_args or sys.argv[1] not in {
        "list",
        "set",
        "set-persistent",
    }:
        print(__doc__)
        sys.exit(1)

    if sys.argv[1] == "list":
        cmd_list()
    elif sys.argv[1] == "set":
        if len(sys.argv) != len_set_sys_args:
            print("Usage: python3 ublox_serial.py set <bus-port> <string>")
            sys.exit(1)
        cmd_set(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == "set-persistent":
        if len(sys.argv) != len_set_sys_args:
            print("Usage: python3 ublox_serial.py set-persistent <bus-port> <string>")
            sys.exit(1)
        cmd_set(sys.argv[2], sys.argv[3], memory_layer=7)


if __name__ == "__main__":
    main()
