#!/usr/bin/env python3
"""
Pi-side receiver for ESP32-CAM UART video stream.

Receives framed JPEG images over serial and saves/displays them.

Protocol:
    [MAGIC 4B: AA 55 01 00] [LENGTH 4B LE] [JPEG data] [CRC16 2B]

Usage:
    uv run python receiver.py snap              # Capture a single frame
    uv run python receiver.py stream            # Stream and save frames
    uv run python receiver.py stream --display  # Stream + open viewer
    uv run python receiver.py status            # Query ESP32 status
    uv run python receiver.py cmd "QUALITY 15"  # Send raw command
"""

import argparse
import os
import struct
import sys
import time
from pathlib import Path

import serial

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 921600
MAGIC = b"\xaa\x55\x01\x00"
HEADER_SIZE = 8  # 4 magic + 4 length
OUTPUT_DIR = Path("frames")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def open_serial() -> serial.Serial:
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)


def send_command(ser: serial.Serial, cmd: str) -> str:
    """Send a command and read the text response line."""
    ser.reset_input_buffer()
    ser.write(f"{cmd}\n".encode())
    line = ser.readline().decode(errors="replace").strip()
    return line


def receive_frame(ser: serial.Serial, timeout: float = 5.0) -> bytes | None:
    """Wait for and receive a single framed JPEG from the ESP32."""
    deadline = time.monotonic() + timeout
    buf = b""

    # Scan for magic bytes
    while time.monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        buf += byte
        if len(buf) > 256 and MAGIC not in buf[-256:]:
            buf = buf[-4:]  # keep sliding window small
        if buf.endswith(MAGIC):
            break
    else:
        return None

    # Read length (4 bytes, little-endian)
    raw_len = ser.read(4)
    if len(raw_len) < 4:
        return None
    frame_len = struct.unpack("<I", raw_len)[0]

    if frame_len > 500_000:  # sanity check
        print(f"[!] Implausible frame size: {frame_len} bytes, skipping")
        return None

    # Read JPEG data
    jpeg_data = b""
    remaining = frame_len
    while remaining > 0 and time.monotonic() < deadline:
        chunk = ser.read(min(remaining, 4096))
        if not chunk:
            break
        jpeg_data += chunk
        remaining -= len(chunk)

    if len(jpeg_data) != frame_len:
        print(f"[!] Incomplete frame: got {len(jpeg_data)}/{frame_len} bytes")
        return None

    # Read and verify CRC
    raw_crc = ser.read(2)
    if len(raw_crc) < 2:
        print("[!] Missing CRC")
        return None

    expected_crc = struct.unpack("<H", raw_crc)[0]
    actual_crc = crc16(jpeg_data)
    if expected_crc != actual_crc:
        print(f"[!] CRC mismatch: expected {expected_crc:#06x}, got {actual_crc:#06x}")
        return None

    return jpeg_data


def cmd_snap(args):
    """Capture a single frame."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    ser = open_serial()

    # Wait for ESP32 to be ready
    time.sleep(0.1)
    resp = send_command(ser, "SNAP")

    # If we got a text response, the frame follows
    frame = receive_frame(ser, timeout=5.0)
    if frame is None:
        # Maybe the response WAS the frame (no text prefix) — retry
        ser.reset_input_buffer()
        send_command(ser, "SNAP")
        frame = receive_frame(ser, timeout=5.0)

    if frame is None:
        print("[!] Failed to capture frame")
        ser.close()
        return 1

    ts = time.strftime("%Y%m%d_%H%M%S")
    path = OUTPUT_DIR / f"snap_{ts}.jpg"
    path.write_bytes(frame)
    print(f"[+] Saved {path} ({len(frame)} bytes)")
    ser.close()
    return 0


def cmd_stream(args):
    """Stream frames continuously."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    ser = open_serial()

    resp = send_command(ser, "START")
    print(f"[*] ESP32: {resp}")
    print(f"[*] Streaming to {OUTPUT_DIR}/ — Ctrl+C to stop")

    frame_count = 0
    start_time = time.monotonic()

    try:
        while True:
            frame = receive_frame(ser, timeout=5.0)
            if frame is None:
                continue

            frame_count += 1
            elapsed = time.monotonic() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0

            if args.save_all:
                path = OUTPUT_DIR / f"frame_{frame_count:06d}.jpg"
                path.write_bytes(frame)

            # Always save latest frame (for external viewers)
            latest = OUTPUT_DIR / "latest.jpg"
            latest.write_bytes(frame)

            print(
                f"\r[*] Frame {frame_count} | {len(frame):,} bytes | {fps:.1f} fps",
                end="",
                flush=True,
            )

    except KeyboardInterrupt:
        print(f"\n[*] Stopped. {frame_count} frames in {time.monotonic() - start_time:.1f}s")
        send_command(ser, "STOP")
    finally:
        ser.close()


def cmd_status(args):
    """Query ESP32 status."""
    ser = open_serial()
    resp = send_command(ser, "STATUS")
    print(f"ESP32: {resp}")
    ser.close()


def cmd_send(args):
    """Send a raw command."""
    ser = open_serial()
    resp = send_command(ser, args.command)
    print(f"ESP32: {resp}")
    ser.close()


def main():
    parser = argparse.ArgumentParser(description="ESP32-CAM UART receiver")
    sub = parser.add_subparsers(dest="action", required=True)

    sub.add_parser("snap", help="Capture a single frame")

    sp_stream = sub.add_parser("stream", help="Stream frames continuously")
    sp_stream.add_argument(
        "--save-all", action="store_true", help="Save every frame (not just latest)"
    )

    sub.add_parser("status", help="Query ESP32 status")

    sp_cmd = sub.add_parser("cmd", help="Send a raw command")
    sp_cmd.add_argument("command", help="Command string to send")

    args = parser.parse_args()

    if args.action == "snap":
        return cmd_snap(args)
    elif args.action == "stream":
        return cmd_stream(args)
    elif args.action == "status":
        return cmd_status(args)
    elif args.action == "cmd":
        return cmd_send(args)


if __name__ == "__main__":
    sys.exit(main() or 0)
