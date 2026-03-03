#!/usr/bin/env python3
"""
ESP32 dev tool for Raspberry Pi.

Manages PlatformIO projects, flashing via GPIO-controlled boot mode,
and serial communication over UART.

Wiring (Pi -> ESP32-WROVER-CAM):
  Pi Pin 2  (5V)      -> ESP32 5V
  Pi Pin 6  (GND)     -> ESP32 GND
  Pi Pin 8  (BCM 14)  -> ESP32 U0RXD (GPIO3)
  Pi Pin 10 (BCM 15)  -> ESP32 U0TXD (GPIO1)
  Pi Pin 11 (BCM 17)  -> ESP32 GPIO0 (Boot Mode)
  Pi Pin 13 (BCM 27)  -> ESP32 RST

Usage:
  uv run esp.py init my-project            # Scaffold new PlatformIO project
  uv run esp.py build esp32-camera         # Build project
  uv run esp.py flash esp32-camera         # Build + flash
  uv run esp.py flash esp32-camera --no-build
  uv run esp.py monitor                    # Interactive serial monitor
  uv run esp.py monitor -b 921600          # Monitor at specific baud
  uv run esp.py reset                      # Reset ESP32
  uv run esp.py erase                      # Erase entire flash
  uv run esp.py check                      # Verify setup
"""

import argparse
import subprocess
import sys
import textwrap
import threading
import time
from pathlib import Path

import RPi.GPIO as GPIO
import serial

# ─── Pin Configuration (BCM numbering) ───
BOOT_PIN = 17   # Pi BCM 17 (Pin 11) -> ESP32 GPIO0
RESET_PIN = 27  # Pi BCM 27 (Pin 13) -> ESP32 RST

# ─── Serial Configuration ───
SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD = 115200
FLASH_BAUD = 460800

# ─── PlatformIO ───
_FW_GLOB = ".pio/build/*/firmware.bin"


# ─── GPIO helpers ───

def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BOOT_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)


def cleanup_gpio():
    GPIO.output(BOOT_PIN, GPIO.HIGH)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    GPIO.cleanup()


def enter_bootloader():
    """Put ESP32 into bootloader mode. Holds GPIO0 LOW until release_bootloader()."""
    print("[*] Entering bootloader mode...")
    GPIO.output(BOOT_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.5)
    # Keep GPIO0 LOW — esptool needs bootloader to stay active.
    # Call release_bootloader() after flashing.


def release_bootloader():
    GPIO.output(BOOT_PIN, GPIO.HIGH)


def reset_esp32():
    print("[*] Resetting ESP32...")
    GPIO.output(BOOT_PIN, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    print("[ok] Reset complete")


# ─── PlatformIO helpers ───

def _find_project(name: str) -> Path:
    candidate = Path(name)
    if not candidate.is_dir():
        candidate = Path(__file__).parent / name
    if not candidate.is_dir():
        print(f"[err] Directory not found: {name}")
        sys.exit(1)
    if not (candidate / "platformio.ini").exists():
        print(f"[err] No platformio.ini in {candidate}")
        sys.exit(1)
    return candidate.resolve()


def _find_build_dir(project_dir: Path) -> Path | None:
    """Find the PlatformIO build env directory containing firmware.bin."""
    bins = sorted(project_dir.glob(_FW_GLOB))
    if not bins:
        return None
    return bins[0].parent


# ─── Commands ───

def cmd_init(args):
    name = args.name
    project_dir = Path(__file__).parent / name

    if project_dir.exists():
        print(f"[err] {project_dir} already exists")
        sys.exit(1)

    print(f"[*] Creating {name}/")
    src = project_dir / "src"
    inc = project_dir / "include"
    src.mkdir(parents=True)
    inc.mkdir(parents=True)

    (project_dir / "platformio.ini").write_text(textwrap.dedent(f"""\
        [env:esp32cam]
        platform = espressif32
        board = esp32cam
        framework = arduino
        monitor_speed = 921600
        upload_speed = 460800

        build_flags =
            -DBOARD_HAS_PSRAM
            -mfix-esp32-psram-cache-issue
    """))

    (src / "main.cpp").write_text(textwrap.dedent("""\
        #include <Arduino.h>

        void setup() {
            Serial.begin(921600);
            Serial.println("OK:ready");
        }

        void loop() {
            if (Serial.available()) {
                String cmd = Serial.readStringUntil('\\n');
                cmd.trim();
                Serial.printf("echo: %s\\n", cmd.c_str());
            }
        }
    """))

    print(f"[ok] Created {name}/")
    print(f"     {name}/platformio.ini")
    print(f"     {name}/src/main.cpp")
    print(f"     {name}/include/")
    print(f"")
    print(f"  Build:  uv run esp.py build {name}")
    print(f"  Flash:  uv run esp.py flash {name}")


def cmd_build(args):
    project_dir = _find_project(args.name)
    print(f"[*] Building {project_dir.name}...")

    result = subprocess.run(
        [sys.executable, "-m", "platformio", "run"],
        cwd=project_dir,
    )
    if result.returncode != 0:
        print("[err] Build failed")
        sys.exit(1)

    build_dir = _find_build_dir(project_dir)
    if build_dir is None:
        print("[err] Build succeeded but no firmware.bin found")
        sys.exit(1)

    fw = build_dir / "firmware.bin"
    print(f"[ok] {fw} ({fw.stat().st_size:,} bytes)")
    return build_dir


def cmd_flash(args):
    if args.no_build:
        project_dir = _find_project(args.name)
        build_dir = _find_build_dir(project_dir)
        if build_dir is None:
            print(f"[err] No compiled binary in {project_dir}")
            print("  Run a build first, or drop --no-build")
            sys.exit(1)
        print(f"[*] Using existing build: {build_dir}")
    else:
        build_dir = cmd_build(args)

    # ESP32 Arduino partition layout:
    #   0x1000  - bootloader.bin
    #   0x8000  - partitions.bin
    #   0x10000 - firmware.bin (application)
    bootloader = build_dir / "bootloader.bin"
    partitions = build_dir / "partitions.bin"
    firmware = build_dir / "firmware.bin"

    flash_args = []
    if bootloader.exists():
        flash_args += ["0x1000", str(bootloader)]
    if partitions.exists():
        flash_args += ["0x8000", str(partitions)]
    flash_args += ["0x10000", str(firmware)]

    setup_gpio()
    try:
        enter_bootloader()

        # Flush any stale data from the serial port
        try:
            ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.1)
            ser.reset_input_buffer()
            ser.close()
        except serial.SerialException:
            pass
        time.sleep(0.2)

        baud = args.baud or FLASH_BAUD
        print(f"[*] Flashing via {SERIAL_PORT} at {baud} baud...")
        for i in range(0, len(flash_args), 2):
            print(f"     {flash_args[i]} <- {Path(flash_args[i+1]).name}")

        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32",
            "--port", SERIAL_PORT,
            "--baud", str(baud),
            "--before", "no_reset",
            "--after", "no_reset",
            "write_flash",
            "--flash_mode", "dio",
            "--flash_freq", "80m",
            "--flash_size", "detect",
        ] + flash_args
        result = subprocess.run(cmd)

        if result.returncode != 0:
            print("[err] Flash failed")
            sys.exit(1)

        print("[ok] Flash complete")
        release_bootloader()
        time.sleep(0.5)
        reset_esp32()
    finally:
        cleanup_gpio()


def cmd_monitor(args):
    baud = args.baud or SERIAL_BAUD

    print(f"[*] Monitor on {SERIAL_PORT} at {baud} baud")
    print("[*] Type to send, Ctrl+C to exit\n")

    try:
        ser = serial.Serial(SERIAL_PORT, baud, timeout=0.1)
        ser.reset_input_buffer()

        running = True

        def reader():
            while running:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    print(data.decode("utf-8", errors="replace"), end="", flush=True)
                time.sleep(0.01)

        t = threading.Thread(target=reader, daemon=True)
        t.start()

        while True:
            line = input()
            ser.write((line + "\n").encode("utf-8"))

    except KeyboardInterrupt:
        print("\n[*] Monitor closed")
    except serial.SerialException as e:
        print(f"[err] Serial error: {e}")
    finally:
        running = False
        if "ser" in locals():
            ser.close()


def cmd_reset(args):
    setup_gpio()
    try:
        reset_esp32()
    finally:
        cleanup_gpio()


def cmd_erase(args):
    setup_gpio()
    try:
        enter_bootloader()
        time.sleep(0.5)

        baud = args.baud or FLASH_BAUD
        print("[*] Erasing flash...")

        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32",
            "--port", SERIAL_PORT,
            "--baud", str(baud),
            "--before", "no_reset",
            "--after", "no_reset",
            "erase_flash",
        ]
        result = subprocess.run(cmd)

        if result.returncode != 0:
            print("[err] Erase failed")
            sys.exit(1)

        print("[ok] Flash erased")
        time.sleep(0.5)
        reset_esp32()
    finally:
        cleanup_gpio()


def cmd_check(args):
    print("[*] Checking setup...\n")

    # esptool
    try:
        r = subprocess.run(
            [sys.executable, "-m", "esptool", "version"],
            capture_output=True, text=True,
        )
        if r.returncode == 0:
            print(f"[ok] esptool: {r.stdout.strip().splitlines()[0]}")
        else:
            print("[err] esptool not working")
    except Exception:
        print("[err] esptool not found")

    # platformio
    try:
        r = subprocess.run(
            [sys.executable, "-m", "platformio", "--version"],
            capture_output=True, text=True,
        )
        if r.returncode == 0:
            print(f"[ok] platformio: {r.stdout.strip()}")
        else:
            print("[err] platformio not working")
    except Exception:
        print("[err] platformio not found")

    # serial
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        ser.close()
        print(f"[ok] Serial port {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"[err] Serial port {SERIAL_PORT}: {e}")

    # gpio
    try:
        setup_gpio()
        print("[ok] GPIO pins")
        cleanup_gpio()
    except Exception as e:
        print(f"[err] GPIO: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="ESP32 dev tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("-p", "--port", default=None, help="Serial port override")
    sub = parser.add_subparsers(dest="cmd", required=True)

    # init
    p = sub.add_parser("init", help="Scaffold a new PlatformIO project")
    p.add_argument("name", help="Project directory name")

    # build
    p = sub.add_parser("build", help="Build a PlatformIO project")
    p.add_argument("name", help="Project directory name")

    # flash
    p = sub.add_parser("flash", help="Build + flash (default) or flash existing binary")
    p.add_argument("name", help="Project directory name")
    p.add_argument("-b", "--baud", type=int, default=None, help="Flash baud rate")
    p.add_argument("--no-build", action="store_true", help="Skip build, use existing binary")

    # monitor
    p = sub.add_parser("monitor", help="Interactive serial monitor")
    p.add_argument("-b", "--baud", type=int, default=None, help="Baud rate")

    # reset
    sub.add_parser("reset", help="Reset ESP32")

    # erase
    p = sub.add_parser("erase", help="Erase entire flash")
    p.add_argument("-b", "--baud", type=int, default=None, help="Baud rate")

    # check
    sub.add_parser("check", help="Verify setup")

    args = parser.parse_args()

    global SERIAL_PORT
    if args.port:
        SERIAL_PORT = args.port

    dispatch = {
        "init": cmd_init,
        "build": cmd_build,
        "flash": cmd_flash,
        "monitor": cmd_monitor,
        "reset": cmd_reset,
        "erase": cmd_erase,
        "check": cmd_check,
    }
    dispatch[args.cmd](args)


if __name__ == "__main__":
    main()
