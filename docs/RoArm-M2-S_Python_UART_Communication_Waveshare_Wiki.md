# RoArm-M2-S Python UART Communication

Source: <https://www.waveshare.com/wiki/RoArm-M2-S_Python_UART_Communication>  
Fetched: 2026-03-02  
Conversion note: mirrored from the page's MediaWiki `action=raw` source. Screenshot/image markup and repeated tutorial-directory links were omitted.

## Purpose

This page explains how to talk to the arm over a serial port from Python running on a PC, Raspberry Pi, or Jetson-class host.

## Demo Package

Waveshare points to this downloadable bundle:

- `RoArm-M2-S_python.zip`
- URL: <https://files.waveshare.com/wiki/RoArm-M2-S/RoArm-M2-S_python.zip>

The page says the bundle contains four Python demos, and the serial demo is `serial_simple_ctrl.py`.

## Environment Setup (Condensed)

The original tutorial is Windows-oriented, but the useful steps are platform-neutral:

1. Unzip the Python demo package.
2. Enter the extracted project directory.
3. Create a virtual environment:

```bash
python -m venv roarmpython-env
```

4. Activate it.
5. Install dependencies from `requirements.txt`:

```bash
python -m pip install -r requirements.txt
```

## `serial_simple_ctrl.py`

Waveshare includes this exact file content:

```python
import serial
import argparse
import threading

def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

def main():
    global ser
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')

    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    try:
        while True:
            command = input("")
            ser.write(command.encode() + b'\n')
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
```

## How Waveshare Says To Use It

The documented flow:

1. Power on the robotic arm.
2. Connect the host to the arm via the Type-C USB port, or use RX/TX directly.
3. Identify the serial port assigned by the host OS.
4. Run the script with that port name.

Windows example from the page:

```bash
python serial_simple_ctrl.py COM20
```

Linux-style port names mentioned by Waveshare:

- `/dev/ttyUSB0`

For macOS, the equivalent will usually be a CP210x-backed device such as:

- `/dev/cu.usbserial-*`
- `/dev/tty.usbserial-*`
- `/dev/cu.SLAB_USBtoUART`

## Serial Protocol Behavior Implied By The Demo

The script tells us the practical protocol shape:

- Serial settings:
  - baud rate `115200`
  - `RTS` forced low
  - `DTR` forced low
- Input:
  - user types raw JSON text
  - script appends `\n`
  - bytes are sent directly without additional framing
- Output:
  - background thread continuously calls `readline()`
  - responses are expected to be newline-terminated UTF-8 text
  - the controller may emit unsolicited startup / status lines as soon as the port opens

## Practical Takeaway For This Repo

If you want the same behavior locally, `serial_simple_ctrl.py` is not a hypothetical helper. Waveshare publishes its contents, and it is just a thin interactive line-oriented serial terminal specialized for JSON commands at `115200`.
