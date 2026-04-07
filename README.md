# robo-arm

Minimal direct-control Python package for the RoArm-M2-S.

## What remains

- `src/roarm/arm.py`: serial JSON interface to the arm firmware
- `src/roarm/__init__.py`: package exports

## Install

```bash
uv sync
```

## Usage

```python
from roarm import Arm

with Arm("/dev/tty.usbserial-0001") as arm:
    state = arm.read_state()
    arm.move((0.0, 0.0, 0.0, 0.0))
```
