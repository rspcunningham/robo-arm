# Plan: Direct Mac-to-Servo Control for RoArm-M2-S

## Context

Replace the entire current stack (Mac → Pi → ROS2 → JSON → ESP32 → servos) with a direct connection (Mac → SCS protocol → servos). The ESP32 gets reflashed as a dumb serial bridge. A new Python package on the Mac speaks the Feetech SCS protocol directly to the ST3215 servos. This gives full register-level control for direct neural policy inference — no middleware, no abstraction layers the user doesn't own.

New standalone repo, separate from `robo-arm`.

---

## Repo Structure

```
roarm-direct/
├── pyproject.toml                  # deps: pyserial
├── firmware/
│   └── serial_bridge/
│       └── serial_bridge.ino       # ESP32 USB↔servo bus bridge (~15 lines)
├── src/roarm/
│   ├── __init__.py                 # re-exports Arm
│   ├── scs.py                      # Layer 1: SCS packet protocol
│   ├── st3215.py                   # Layer 2: register map + per-servo interface
│   └── arm.py                      # Layer 3: arm-level interface (4 joints, 5 servos)
├── tools/
│   ├── scan.py                     # scan bus for servo IDs
│   ├── read_state.py               # continuous state readout
│   └── move.py                     # CLI joint mover
└── examples/
    └── control_loop.py             # minimal policy integration
```

---

## Part 1: ESP32 Bridge Firmware

`firmware/serial_bridge/serial_bridge.ino`

- Serial0 (USB/CP2102N) at 1 Mbps
- Serial1 (GPIO 18 TX, GPIO 19 RX) at 1 Mbps to servo bus
- `loop()`: forward bytes both directions
- Board's existing half-duplex circuit handles bus direction
- Flash via Arduino IDE, board = "ESP32 Dev Module"
- **Keep a backup of the factory firmware binary before flashing**

---

## Part 2: Python Package

### Layer 1 — SCS Protocol (`scs.py`)

Class `SCSBus(port, baudrate=1_000_000)`:
- `_build_packet(id, instruction, params)` → `0xFF 0xFF <ID> <LEN> <INSTR> <PARAMS> <CHECKSUM>`
- `_send(packet)` / `_receive(id)` → parse response, validate checksum
- `ping(id) → bool`
- `read(id, addr, length) → bytes`
- `write(id, addr, data)`
- `sync_write(addr, length, {id: data, ...})` → single broadcast packet
- `reg_write(id, addr, data)` + `action()` → buffered simultaneous execution
- Configurable `echo_discard` flag in case the bus echoes sent bytes

### Layer 2 — ST3215 Registers (`st3215.py`)

Register constants (key ones):
- **Writable**: torque_enable(40), acceleration(41), goal_position(42), goal_time(44), goal_speed(46), torque_limit(48)
- **Readable**: present_position(56), present_speed(58), present_load(60), present_voltage(62), present_temperature(63), present_status(65), moving(66), present_current(69)

Class `ST3215Servo(bus, id)`:
- `read_state() → dict` — bulk read addresses 56–70 (15 bytes, one round-trip)
- `set_goal_position(pos, speed, accel)`
- `set_torque(enabled)`, `set_torque_limit(limit)`
- Data encoding: little-endian u16, sign bits (bit 15 for speed, bit 10 for load)

### Layer 3 — Arm Interface (`arm.py`)

Servo mapping: base=11, shoulder_drive=12, shoulder_driven=13, elbow=14, gripper=15

```python
@dataclass
class ArmState:
    position: dict[str, float]      # radians
    velocity: dict[str, float]      # rad/s
    load: dict[str, float]          # normalized
    voltage: float                  # volts
    temperature: dict[str, int]     # celsius
    timestamp: float
```

Class `Arm(port)`:
- `connect()` — open serial, ping all 5 servos
- `read_state() → ArmState` — bulk read all servos (~2.5ms)
- `command(base, shoulder, elbow, gripper)` — sync_write to all 5 servos (shoulder_drive + shoulder_driven commanded together, one may be mirrored)
- `torque_enable()` / `torque_disable()`
- `set_torque_limits(limit=500)` — default 50% for dev safety
- `home()` — move to zero pose
- Context manager: `__enter__` connects + enables torque, `__exit__` disables + disconnects

Radian ↔ position: 4096 steps / 360° = ~651.9 steps/rad. Per-joint calibration: center offset + direction sign, determined empirically.

---

## Implementation Order

1. **`scs.py`** — protocol foundation
2. **Flash ESP32 bridge** — verify with `tools/scan.py` (ping all IDs)
3. **`st3215.py`** — register map + bulk reads. Verify with `tools/read_state.py`
4. **Calibration** — read raw positions at known poses, determine center offsets + direction signs + shoulder mirror relationship
5. **`arm.py`** — full arm interface with calibration values
6. **Tools + example** — `move.py`, `control_loop.py`

---

## Things to watch for

- **USB latency**: CP2102N + macOS USB polling adds ~1-2ms per transaction. Realistic full loop (read + write) is 10-15ms → 50-100 Hz
- **Echo**: if the half-duplex circuit echoes sent bytes, discard them before reading responses
- **Shoulder dual-drive**: servos 12 and 13 may need mirrored commands depending on mounting — empirical verification needed
- **CP2102N at 1 Mbps**: should work per spec, but fallback is 500kbaud (requires reconfiguring servo EEPROM baud register)
- **macOS serial driver**: use Silicon Labs CP210x driver, not Apple's built-in, for lower latency

---

## Performance Budget

| Operation | Time |
|-----------|------|
| sync_write 5 servos | ~0.2ms wire |
| bulk read 1 servo | ~0.5ms |
| read_state (5 servos) | ~2.5ms |
| USB overhead per transaction | ~1-2ms |
| **Full control loop** | **~10-15ms (50-100 Hz)** |
