# RoArm-M2-S JSON Command Families

Sources:

- <https://www.waveshare.com/wiki/RoArm-M2-S_Robotic_Arm_Control>
- <https://www.waveshare.com/wiki/RoArm-M2-S_EoAT_Setting>
- <https://www.waveshare.com/wiki/RoArm-M2-S_WIFI_Configuration>
- <https://www.waveshare.com/wiki/RoArm-M2-S_FLASH_File_System_Operation>
- <https://www.waveshare.com/wiki/RoArm-M2-S_Step_Recording_and_Reproduction>
- <https://www.waveshare.com/wiki/RoArm-M2-S_ESP-NOW_Control>
- <https://www.waveshare.com/wiki/RoArm-M2-S_Python_HTTP_Request_Communication>

Fetched: 2026-03-02  
Conversion note: condensed from each page's MediaWiki `action=raw` source. Repeated tutorial-directory blocks, screenshots, and boilerplate were omitted.

## Movement Control

Source: `RoArm-M2-S_Robotic_Arm_Control`

### Reset

```json
{"T":100}
```

- `T: 100` is `CMD_MOVE_INIT`.
- Moves all joints to the initial position.
- Waveshare notes this can block until motion completes.

### Single Joint (Radians)

```json
{"T":101,"joint":1,"rad":0,"spd":0,"acc":10}
```

- `joint`:
  - `1`: base
  - `2`: shoulder
  - `3`: elbow
  - `4`: end-effector
- `rad`: target angle in radians.
- `spd`: servo speed in steps/second. `0` means maximum.
- `acc`: acceleration scalar. `0` means maximum.

### All Joints (Radians)

```json
{"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":3.14,"spd":0,"acc":10}
```

- `base`, `shoulder`, `elbow`, `hand` are target joint angles in radians.
- `spd` and `acc` match `T:101`.

### Single Axis IK

```json
{"T":103,"axis":2,"pos":0,"spd":0.25}
```

- `axis`:
  - `1`: x
  - `2`: y
  - `3`: z
  - `4`: t (end-effector angle)
- `pos`:
  - For `x`, `y`, `z`: millimeters
  - For `t`: radians
- `spd`: motion speed coefficient.
- Waveshare notes this can block.

### XYZT Goal IK

```json
{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}
```

- `x`, `y`, `z`: millimeters
- `t`: end-effector angle in radians
- `spd`: speed coefficient
- This path uses interpolation and may block.

### XYZT Direct IK

```json
{"T":1041,"x":235,"y":0,"z":234,"t":3.14}
```

- Same position fields as `T:104`.
- No interpolation.
- Intended for continuous streaming updates.
- Does not block the same way `T:104` can.

### Pose / Joint Feedback

```json
{"T":105}
```

The wiki shows this sample response:

```json
{"T":1051,"x":320.4340313,"y":-3.440909783,"z":212.3634387,"b":-0.010737866,"s":0.047553404,"e":1.598407981,"t":3.136990711,"torB":44,"torS":140,"torE":88,"torH":-20,"torswitchB":1,"torswitchS":1,"torswitchE":1,"torswitchH":1,"v":1211}
```

The wiki assigns these meanings:

- `x`, `y`, `z`: end-effector coordinates
- `b`, `s`, `e`, `t`: joint angles in radians
- `torB`, `torS`, `torE`, `torH`: joint loads
- `torswitch*`: torque lock state
- `v`: voltage in `0.01V` units

Important: the current upstream firmware mirror in `roarm_m2` does emit `T:1051`, `x`, `y`, `z`, `b`, `s`, `e`, `t`, `torB`, `torS`, `torE`, `torH`, but it does not currently show `torswitch*` or `v` in the implementation we inspected. Treat those two fields as firmware-version-dependent.

### End-Effector Angle

```json
{"T":106,"cmd":3.14,"spd":0,"acc":0}
```

- Controls the clamp/wrist joint angle directly in radians.
- `cmd`: target radian angle.

### Single Joint (Degrees)

```json
{"T":121,"joint":1,"angle":0,"spd":10,"acc":10}
```

- Degree-based absolute joint control.
- `spd`: degrees/second
- `acc`: degrees/second squared

### All Joints (Degrees)

```json
{"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}
```

- Degree-based multi-joint command.

### Continuous Motion

```json
{"T":123,"m":0,"axis":0,"cmd":0,"spd":0}
```

- `m`:
  - `0`: angle mode
  - `1`: coordinate mode
- `axis`:
  - In angle mode: `1..4` map to joints
  - In coordinate mode: `1..4` map to x, y, z, hand
- `cmd`:
  - `0`: stop
  - `1`: increase
  - `2`: decrease

## EoAT Configuration

Source: `RoArm-M2-S_EoAT_Setting`

### EoAT Type

```json
{"T":1,"mode":0}
{"T":1,"mode":1}
```

- `mode: 0`: clamp
- `mode: 1`: wrist

### Persist EoAT Type At Boot

```json
{"T":222,"name":"boot","step":"{\"T\":1,\"mode\":1}"}
```

- Appends a boot mission step that sets wrist mode on startup.

### Wrist Geometry

```json
{"T":2,"pos":3,"ea":0,"eb":20}
```

- `pos`: mount-hole selection
- `ea`, `eb`: geometric offsets in millimeters
- Only applies when EoAT mode is wrist.

### Clamp Torque Limit

```json
{"T":107,"tor":200}
```

- `tor` ranges up to `1000`.
- `200` means roughly 20% of full servo torque.

## Wi-Fi Commands

Source: `RoArm-M2-S_WIFI_Configuration`

### Set Wi-Fi Mode On Boot

```json
{"T":401,"cmd":3}
```

- `cmd`:
  - `0`: off
  - `1`: AP
  - `2`: STA
  - `3`: AP+STA

### Set AP Mode

```json
{"T":402,"ssid":"RoArm-M2","password":"12345678"}
```

### Set STA Mode

```json
{"T":403,"ssid":"yourWiFiName","password":"yourWiFiPassword"}
```

### Set AP+STA Mode

```json
{"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"yourWiFiName","sta_password":"yourWiFiPassword"}
```

The wiki text contains a typo (`ap_pawword`), but the firmware header and implementation use `ap_password`.

### Get Wi-Fi Status

```json
{"T":405}
```

Documented response shape:

```json
{"ip":"192.168.10.90","rssi":-50,"wifi_mode_on_boot":3,"sta_ssid":"yourWiFiName","sta_password":"yourWiFiPassword","ap_ssid":"RoArm-M2","ap_password":"12345678"}
```

### Persist Current Wi-Fi Config

```json
{"T":406}
```

### Persist Provided Wi-Fi Config

```json
{"T":407,"mode":3,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"yourWiFiName","sta_password":"yourWiFiPassword"}
```

### Turn Wi-Fi Off

```json
{"T":408}
```

## Flash File Operations

Source: `RoArm-M2-S_FLASH_File_System_Operation`

### Scan Files

```json
{"T":200}
```

### Create File

```json
{"T":201,"name":"file.txt","content":"inputContentHere."}
```

### Read File

```json
{"T":202,"name":"mission_a.mission"}
```

### Delete File

```json
{"T":203,"name":"file.txt"}
```

### Append Line

```json
{"T":204,"name":"file.txt","content":"inputContentHere."}
```

### Insert Line

```json
{"T":205,"name":"file.txt","lineNum":3,"content":"content"}
```

### Replace Line

```json
{"T":206,"name":"file.txt","lineNum":3,"content":"Content"}
```

### Read One Line

```json
{"T":207,"name":"file.txt","lineNum":3}
```

### Delete One Line

```json
{"T":208,"name":"file.txt","lineNum":3}
```

## Mission / Step Playback

Source: `RoArm-M2-S_Step_Recording_and_Reproduction`

Mission files omit the `.mission` suffix in these commands.

### Create Mission

```json
{"T":220,"name":"mission_a","intro":"test mission created in flash."}
```

### Read Mission

```json
{"T":221,"name":"mission_a"}
```

### Append Raw JSON Step

```json
{"T":222,"name":"mission_a","step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}
```

### Append Current-Pose Step

```json
{"T":223,"name":"mission_a","spd":0.25}
```

### Append Delay

```json
{"T":224,"name":"mission_a","delay":3000}
```

### Insert Raw JSON Step

```json
{"T":225,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}
```

### Insert Current-Pose Step

```json
{"T":226,"name":"mission_a","stepNum":3,"spd":0.25}
```

### Insert Delay

```json
{"T":227,"name":"mission_a","stepNum":3,"delay":3000}
```

The wiki page incorrectly labels the last field as `spd` in one example. The firmware header uses `delay`.

### Replace Raw JSON Step

```json
{"T":228,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}
```

### Replace Current-Pose Step

```json
{"T":229,"name":"mission_a","stepNum":3,"spd":0.25}
```

### Replace Delay

```json
{"T":230,"name":"mission_a","stepNum":3,"delay":3000}
```

### Delete Step

```json
{"T":231,"name":"mission_a","stepNum":3}
```

### Execute One Step

```json
{"T":241,"name":"mission_a","stepNum":3}
```

### Play Mission

```json
{"T":242,"name":"mission_a","times":3}
```

- `times: -1` loops forever.

## ESP-NOW Commands

Source: `RoArm-M2-S_ESP-NOW_Control`

### Data Structure

The wiki defines the ESP-NOW payload structure as:

```c
typedef struct struct_message {
  byte devCode;
  float base;
  float shoulder;
  float elbow;
  float hand;
  byte cmd;
  char message[210];
} struct_message;
```

### Broadcast Follower Mode

```json
{"T":300,"mode":1}
{"T":300,"mode":0,"mac":"00:00:00:00:00:00"}
```

### ESP-NOW Mode

```json
{"T":301,"mode":3}
```

- `0`: off
- `1`: stream multicast leader
- `2`: stream unicast / broadcast leader
- `3`: follower

### Get Device MAC

```json
{"T":302}
```

### Add Follower

```json
{"T":303,"mac":"44:17:93:EE:FD:70"}
```

### Remove Follower

```json
{"T":304,"mac":"44:17:93:EE:FD:70"}
```

### Multicast

```json
{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
```

Or send an embedded non-blocking JSON command:

```json
{"T":305,"dev":0,"b":0,"s":0,"e":0,"h":0,"cmd":1,"megs":"{\"T\":114,\"led\":255}"}
```

### Unicast / Broadcast

```json
{"T":306,"mac":"44:17:93:EE:FD:70","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
```

Broadcast uses:

```json
{"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
```

## HTTP Transport

Source: `RoArm-M2-S_Python_HTTP_Request_Communication`

The HTTP transport sends the same JSON commands via:

```text
GET /js?json=<raw-json-string>
```

Waveshare's example client uses:

```python
url = "http://" + ip_addr + "/js?json=" + command
```

Typical targets:

- `192.168.4.1` in AP mode
- the OLED-displayed IP in STA mode

## Known Documentation Inconsistencies

These matter if you are building a ROS2 driver against the real firmware:

- The Wi-Fi page contains a typo: `ap_pawword`; the firmware expects `ap_password`.
- The mission page shows `{"T":227,...,"spd":3000}` in one example; the firmware header defines that field as `delay`.
- The movement docs and the firmware agree that `T:111` is a delay command, not torque control.
- Torque lock is `T:210`, not `T:111`.
