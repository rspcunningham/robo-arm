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

## Dual-arm discovery

```bash
uv run roarm
```

This resolves:

- `acff0192339bef11b2e7b69061ce3355` as `left`
- `24ee9dd6f400f0119b83c3295c2a50c9` as `right`
