# RoArm-M2-S JSON Command Meaning

Source: <https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning>  
Fetched: 2026-03-02  
Conversion note: mirrored from the page's MediaWiki `action=raw` source. Boilerplate navigation, image markup, and tutorial-directory lists were omitted.

## What's JSON

JSON (JavaScript Object Notation) is the data format Waveshare uses to control the arm over its exposed interfaces.

Example command:

```json
{"T":1041,"x":235,"y":0,"z":234,"t":3.14}
```

Waveshare's explanation:

- `T` is the command type.
- `1041` is `CMD_XYZT_DIRECT_CTRL`.
- `x`, `y`, `z`, and `t` are the end-effector coordinates and the clamp/wrist angle.

## Why Waveshare Uses JSON Commands

The page's rationale, condensed:

- Human-readable and easy to debug.
- Easy to parse from common programming environments.
- Cross-platform and language-agnostic.
- Extensible key/value structure.
- Fits naturally with web and API integrations.

## Supported Communication Modes

RoArm-M2-S supports multiple JSON transport methods:

- UART over RX/TX pins
- USB serial over the Type-C port
- HTTP over Wi-Fi
- ESP-NOW

### UART/USB Communication

Waveshare's key details:

- Wired connection
- Default baud rate: `115200`
- Bidirectional communication
- Intended for PCs, Raspberry Pi, Jetson, and similar hosts
- Python demo file: `serial_simple_ctrl.py`

Waveshare includes this exact demo:

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

Protocol details implied by that demo:

- Open the serial port at `115200`.
- Read newline-delimited text responses with `readline()`.
- Send one JSON object per line.
- Append a newline (`\n`) after each command.
- The arm sends text feedback asynchronously on the same serial stream.

### HTTP Request Communication

Waveshare also includes a simple HTTP JSON demo:

```python
import requests
import argparse


def main():
    parser = argparse.ArgumentParser(description='Http JSON Communication')
    parser.add_argument('ip', type=str, help='IP address: 192.168.10.104')

    args = parser.parse_args()

    ip_addr = args.ip

    try:
        while True:
            command = input("input your json cmd: ")
            url = "http://" + ip_addr + "/js?json=" + command
            response = requests.get(url)
            content = response.text
            print(content)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
```

## JSON Command Table (Relevant Excerpts From This Page)

This page only defines a subset of commands directly. Movement-related commands are delegated to other Waveshare pages.

### Torque Lock

```json
{"T":210,"cmd":0}
```

- `T: 210` is `CMD_TORQUE_CTRL`.
- `cmd: 0` turns torque lock off.
- `cmd: 1` turns torque lock on.
- Waveshare notes torque lock is automatically re-enabled when other movement commands are received.

### Dynamic External Force Adaptation

Enable:

```json
{"T":112,"mode":1,"b":60,"s":110,"e":50,"h":50}
```

Disable / reset torque limits:

```json
{"T":112,"mode":0,"b":1000,"s":1000,"e":1000,"h":1000}
```

- `T: 112` is `CMD_DYNAMIC_ADAPTATION`.
- `mode: 0` disables the function.
- `mode: 1` enables the function.
- `b`, `s`, `e`, `h` are torque limits for base, shoulder, elbow, and gripper/wrist.

### Joint PID

```json
{"T":108,"joint":3,"p":16,"i":0}
```

- `T: 108` is `CMD_SET_JOINT_PID`.
- `joint` values on this page:
  - `1`: base
  - `2`: shoulder
  - `3`: elbow
  - `4`: end-effector
- `p` is proportional gain.
- `i` is integral gain.

Reset PID:

```json
{"T":109}
```

- `T: 109` is `CMD_RESET_PID`.

## What This Page Does Not Define

This page explicitly points elsewhere for:

- mechanical movement control
- end-effector control
- Wi-Fi configuration
- ESP-NOW configuration

That means common movement commands such as `{"T":104}`, `{"T":101,...}`, and `{"T":1041,...}` are referenced by Waveshare elsewhere, not fully documented in this specific page.
