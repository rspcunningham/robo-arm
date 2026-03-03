# RoArm-M2-S JSON Command Reference

Sources:

- Upstream firmware repo: `waveshareteam/roarm_m2`
- Reviewed files:
  - `RoArm-M2_example/json_cmd.h`
  - `RoArm-M2_example/uart_ctrl.h`
  - `RoArm-M2_example/RoArm-M2_module.h`
- Supporting wiki mirrors in this `docs/` folder

Generated: 2026-03-02  
Purpose: authoritative request-side command map for implementing a host driver.

## Transport Contract

### UART / USB Serial

- Serial speed: `115200`
- Framing: one JSON object per line
- Terminator: newline (`\n`)
- Parsing in firmware occurs only when a newline is received

Minimal example:

```text
{"T":105}\n
```

### HTTP

The same commands can be sent over HTTP:

```text
GET /js?json=<raw-json>
```

## Command Ranges

- `0..199`: motion, EoAT, device control
- `200..299`: flash files and missions
- `300..399`: ESP-NOW
- `400..499`: Wi-Fi
- `500..599`: servo configuration
- `600..699`: ESP32 / system control

## Core Commands

### Emergency / System State

- `0` `CMD_EMERGENCY_STOP`
  - Request: `{"T":0}`
- `999` `CMD_RESET_EMERGENCY`
  - Request: `{"T":999}`

### EoAT

- `1` `CMD_EOAT_TYPE`
  - Request: `{"T":1,"mode":0}` or `{"T":1,"mode":1}`
  - `mode`: `0` clamp, `1` wrist
- `2` `CMD_CONFIG_EOAT`
  - Request: `{"T":2,"pos":3,"ea":0,"eb":20}`
  - `pos`: mount-hole selector
  - `ea`, `eb`: millimeter offsets
- `107` `CMD_EOAT_GRAB_TORQUE`
  - Request: `{"T":107,"tor":200}`

### Motion

- `100` `CMD_MOVE_INIT`
  - `{"T":100}`
- `101` `CMD_SINGLE_JOINT_CTRL`
  - `{"T":101,"joint":1,"rad":0,"spd":0,"acc":10}`
- `102` `CMD_JOINTS_RAD_CTRL`
  - `{"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":1.57,"spd":0,"acc":10}`
- `103` `CMD_SINGLE_AXIS_CTRL`
  - `{"T":103,"axis":2,"pos":0,"spd":0.25}`
- `104` `CMD_XYZT_GOAL_CTRL`
  - `{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}`
- `1041` `CMD_XYZT_DIRECT_CTRL`
  - `{"T":1041,"x":235,"y":0,"z":234,"t":3.14}`
- `105` `CMD_SERVO_RAD_FEEDBACK`
  - `{"T":105}`
- `106` `CMD_EOAT_HAND_CTRL`
  - `{"T":106,"cmd":3.14,"spd":0,"acc":0}`
- `108` `CMD_SET_JOINT_PID`
  - `{"T":108,"joint":3,"p":16,"i":0}`
- `109` `CMD_RESET_PID`
  - `{"T":109}`
- `110` `CMD_SET_NEW_X`
  - `{"T":110,"xAxisAngle":0}`
- `111` `CMD_DELAY_MILLIS`
  - `{"T":111,"cmd":3000}`
- `112` `CMD_DYNAMIC_ADAPTATION`
  - `{"T":112,"mode":1,"b":60,"s":110,"e":50,"h":50}`
- `113` `CMD_SWITCH_CTRL`
  - `{"T":113,"pwm_a":-255,"pwm_b":-255}`
- `114` `CMD_LIGHT_CTRL`
  - `{"T":114,"led":255}`
- `115` `CMD_SWITCH_OFF`
  - `{"T":115}`
- `121` `CMD_SINGLE_JOINT_ANGLE`
  - `{"T":121,"joint":1,"angle":0,"spd":10,"acc":10}`
- `122` `CMD_JOINTS_ANGLE_CTRL`
  - `{"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}`
- `123` `CMD_CONSTANT_CTRL`
  - `{"T":123,"m":0,"axis":0,"cmd":0,"spd":3}`

### Torque Lock

- `210` `CMD_TORQUE_CTRL`
  - `{"T":210,"cmd":0}`
  - `cmd: 0` off
  - `cmd: 1` on

This is the correct torque command. `T:111` is delay, not torque.

## File Commands

- `200` `CMD_SCAN_FILES`
  - `{"T":200}`
- `201` `CMD_CREATE_FILE`
  - `{"T":201,"name":"file.txt","content":"inputContentHere."}`
- `202` `CMD_READ_FILE`
  - `{"T":202,"name":"file.txt"}`
- `203` `CMD_DELETE_FILE`
  - `{"T":203,"name":"file.txt"}`
- `204` `CMD_APPEND_LINE`
  - `{"T":204,"name":"file.txt","content":"inputContentHere."}`
- `205` `CMD_INSERT_LINE`
  - `{"T":205,"name":"file.txt","lineNum":3,"content":"content"}`
- `206` `CMD_REPLACE_LINE`
  - `{"T":206,"name":"file.txt","lineNum":3,"content":"Content"}`
- `207` `CMD_READ_LINE`
  - `{"T":207,"name":"file.txt","lineNum":3}`
- `208` `CMD_DELETE_LINE`
  - `{"T":208,"name":"file.txt","lineNum":3}`

## Mission Commands

- `220` `CMD_CREATE_MISSION`
  - `{"T":220,"name":"mission_a","intro":"test mission created in flash."}`
- `221` `CMD_MISSION_CONTENT`
  - `{"T":221,"name":"mission_a"}`
- `222` `CMD_APPEND_STEP_JSON`
  - `{"T":222,"name":"mission_a","step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}`
- `223` `CMD_APPEND_STEP_FB`
  - `{"T":223,"name":"mission_a","spd":0.25}`
- `224` `CMD_APPEND_DELAY`
  - `{"T":224,"name":"mission_a","delay":3000}`
- `225` `CMD_INSERT_STEP_JSON`
  - `{"T":225,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}`
- `226` `CMD_INSERT_STEP_FB`
  - `{"T":226,"name":"mission_a","stepNum":3,"spd":0.25}`
- `227` `CMD_INSERT_DELAY`
  - `{"T":227,"name":"mission_a","stepNum":3,"delay":3000}`
- `228` `CMD_REPLACE_STEP_JSON`
  - `{"T":228,"name":"mission_a","stepNum":3,"step":"{\"T\":114,\"led\":255}"}`
- `229` `CMD_REPLACE_STEP_FB`
  - `{"T":229,"name":"mission_a","stepNum":3,"spd":0.25}`
- `230` `CMD_REPLACE_DELAY`
  - `{"T":230,"name":"mission_a","stepNum":3,"delay":3000}`
- `231` `CMD_DELETE_STEP`
  - `{"T":231,"name":"mission_a","stepNum":3}`
- `241` `CMD_MOVE_TO_STEP`
  - `{"T":241,"name":"mission_a","stepNum":3}`
- `242` `CMD_MISSION_PLAY`
  - `{"T":242,"name":"mission_a","times":3}`

## ESP-NOW Commands

- `300` `CMD_BROADCAST_FOLLOWER`
  - `{"T":300,"mode":1}`
  - `{"T":300,"mode":0,"mac":"CC:DB:A7:5B:E4:1C"}`
- `301` `CMD_ESP_NOW_CONFIG`
  - `{"T":301,"mode":3}`
- `302` `CMD_GET_MAC_ADDRESS`
  - `{"T":302}`
- `303` `CMD_ESP_NOW_ADD_FOLLOWER`
  - `{"T":303,"mac":"FF:FF:FF:FF:FF:FF"}`
- `304` `CMD_ESP_NOW_REMOVE_FOLLOWER`
  - `{"T":304,"mac":"FF:FF:FF:FF:FF:FF"}`
- `305` `CMD_ESP_NOW_GROUP_CTRL`
  - `{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}`
- `306` `CMD_ESP_NOW_SINGLE`
  - `{"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}`

## Wi-Fi Commands

- `401` `CMD_WIFI_ON_BOOT`
  - `{"T":401,"cmd":3}`
- `402` `CMD_SET_AP`
  - `{"T":402,"ssid":"RoArm-M2","password":"12345678"}`
- `403` `CMD_SET_STA`
  - `{"T":403,"ssid":"JSBZY-2.4G","password":"waveshare0755"}`
- `404` `CMD_WIFI_APSTA`
  - `{"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}`
- `405` `CMD_WIFI_INFO`
  - `{"T":405}`
- `406` `CMD_WIFI_CONFIG_CREATE_BY_STATUS`
  - `{"T":406}`
- `407` `CMD_WIFI_CONFIG_CREATE_BY_INPUT`
  - `{"T":407,"mode":3,"ap_ssid":"RoArm-M2","ap_password":"12345678","sta_ssid":"JSBZY-2.4G","sta_password":"waveshare0755"}`
- `408` `CMD_WIFI_STOP`
  - `{"T":408}`

## Servo Configuration Commands

- `501` `CMD_SET_SERVO_ID`
  - `{"T":501,"raw":1,"new":11}`
- `502` `CMD_SET_MIDDLE`
  - `{"T":502,"id":11}`
- `503` `CMD_SET_SERVO_PID`
  - `{"T":503,"id":14,"p":16}`

## ESP32 / Device Commands

- `600` `CMD_REBOOT`
  - `{"T":600}`
- `601` `CMD_FREE_FLASH_SPACE`
  - `{"T":601}`
- `602` `CMD_BOOT_MISSION_INFO`
  - `{"T":602}`
- `603` `CMD_RESET_BOOT_MISSION`
  - `{"T":603}`
- `604` `CMD_NVS_CLEAR`
  - `{"T":604}`
- `605` `CMD_INFO_PRINT`
  - `{"T":605,"cmd":1}`
  - `cmd: 2` flow feedback
  - `cmd: 1` serial debug info
  - `cmd: 0` silent serial debug mode

## Known Response Payloads

### Motion Feedback (`T:105` Request)

The current upstream firmware implementation emits:

```json
{"T":1051,"x":<mm>,"y":<mm>,"z":<mm>,"b":<rad>,"s":<rad>,"e":<rad>,"t":<rad>,"torB":<int>,"torS":<int>,"torE":<int>,"torH":<int>}
```

Field meaning:

- `x`, `y`, `z`: end-effector position in millimeters
- `b`, `s`, `e`, `t`: joint angles in radians
- `torB`, `torS`, `torE`, `torH`: load values

### Wi-Fi Status (`T:405` Request)

From the Wi-Fi implementation and docs:

```json
{"ip":"<ip>","wifi_mode_on_boot":<0-3>,"sta_ssid":"...","sta_password":"...","ap_ssid":"...","ap_password":"..."}
```

The docs also show `rssi`; that may be firmware-version-dependent.

### Free Flash Space (`T:601` Request)

The firmware populates:

```json
{"info":"free flash space","total":<bytes>,"free":<bytes>}
```

## Notes For A Host Driver

- Do not assume every command is request/response.
- Many commands just trigger side effects and may only emit debug text unless `T:605` is set appropriately.
- For robust parsing, disable debug serial chatter first with `{"T":605,"cmd":0}`.
- Treat `T:105` as the primary structured feedback query for state synchronization.
