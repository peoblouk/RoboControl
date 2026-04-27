<!-- @format -->

# RoboControl

<p align="right">
  <img src="spiffs/web/robocontrol.ico" alt="RoboControl icon" width="50" />
</p>

RoboControl je firmware pro rizeni 6DOF robotickeho ramene na ESP32-S3.
Aktualni verze kombinuje:

- Web UI + WebSocket (`/ws`)
- UART konzoli pres `esp_console` REPL
- planner pohybu s HOME/reference workflow
- CAN/TWAI ridici protokol pro nadrizeny system
- onboard RGB LED (WS2812) jako stavovou signalizaci

---

## Co je hotove

- Rizeni 6 kloubu (+ follower servo pro J1 a gripper kanal)
- IK v WORK souradnicich (`move`, `move_xyz`) s offsetem
- HOME prikaz, ktery po dojeti nastavuje referenci
- G-code pipeline (`run`, `line`, `stop`, `reset`, `sync`)
- SoftAP HTTP server + WebSocket telemetrie
- SPIFFS file manager (`/spiffs/data`) pro `.txt` a `.gcode`
- CAN upload/spousteni programu po slotech (`can_slot_0..3.gcode`)
- Stavova RGB LED na ESP32-S3-DevKitC-1

---

## Struktura projektu

```text
main/
|-- main.c
|-- config.h
|-- can_communication/
|   |-- can_communication.c
|   `-- can_communication.h
|-- robot_io/
|   |-- robot_io.c
|   `-- robot_io.h
|-- wifi_server/
|   |-- wifi_server.c
|   |-- wifi_server.h
|   |-- http_handlers.c
|   |-- ws_handlers.c
|   |-- file_manager.c
|   `-- *.h
|-- cmd_control/
|   |-- cmd_control.c
|   `-- cmd_control.h
|-- gcode/
|   |-- gcode.c
|   `-- gcode.h
|-- status_led/
|   |-- status_led.c
|   `-- status_led.h
|-- rt_stats/
|   |-- rt_stats.c
|   `-- rt_stats.h
`-- certs/
    |-- servercert.pem
    `-- prvtkey.pem
```

---

## Konfigurace uzlu

Hlavni volby jsou v `main/config.h`:

- `ROBOT_NODE_ID` (aktualne `2`)
- `WIFI_SSID = "RoboControl_<ROBOT_NODE_ID>"`
- `CAN_NODE_ID = ROBOT_NODE_ID & 0x7F`
- `CAN_BITRATE = 500000`
- `CAN_NO_ACK_MODE = 1` (NO_ACK)

---

## Hardware map

<p align="center">
  <img src="img/RobotControl_3D.png" alt="3D Render" width="700" />
</p>

### Serva (LEDC PWM)

| Servo ID | GPIO | Poznamka |
| --- | --- | --- |
| 0 | 35 | J0 |
| 1 | 36 | J1 master |
| 2 | 37 | J1 follower |
| 3 | 39 | J2 |
| 4 | 40 | J3 |
| 5 | 41 | J4 |
| 6 | 42 | J5 / gripper |

### CAN / TWAI

- TX: `GPIO14`
- RX: `GPIO13`
- bitrate: `500 kbps`

### Senzory (ADC mapa v konfiguraci)

Mapa ADC je pripravena pro 6 senzoru, ale default build bezi v sensorless rezimu:

- `SENSOR_COUNT = 0`
- `sensors_init()` se v `main.c` vola jen kdyz `SENSOR_COUNT > 0`
- WebSocket telemetrie posila aktualne odhadnute uhly kloubu (`robot_get_est_angle`)

### Onboard RGB LED

- WS2812 data pin: `GPIO38` (`STATUS_LED_WS2812_GPIO`)
- `DISARMED` = cervena
- `READY/UNREFERENCED` = zelena
- `RUNNING` = oranzova blikani

---

## Robot stavy a bezpecnost

- `disarm` vypne PWM vystupy, zastavi program, zahodi reference.
- `arm` znovu povoli pohyb a vymaze error stav.
- Prikazy `joint` a `move` jsou blokovane, kdyz je robot disarmed.
- `move` pracuje ve WORK souradnicich.
- `home` frontuje HOME segment; reference se nastavi az po jeho dokonceni.

Aktualni systemove stavy (`robot_get_system_state_name()`):

- `DISARMED`
- `UNREFERENCED`
- `READY`
- `RUNNING`
- `READY_FOR_SYNC`
- `ERROR`

---

## Konzole (UART REPL)

Konzole bezi pres `esp_console`:

- prompt: `>>`
- historie prikazu: `1`
- `linenoise` je v dumb mode (`linenoiseSetDumbMode(1)`)

### Dostupne prikazy

- `joint <id> <angle>`
- `move <x> <y> <z> [pitch]`
- `home`
- `arm`
- `disarm`
- `state`
- `wcofs [x y z]`
- `gcode run <file>`
- `gcode line <...>`
- `gcode stop`
- `gcode reset`
- `gcode sync`
- `can up`
- `can status`
- `can hb`
- `can probe [count] [gap_ms]`
- `can tx <id> [b0 ... b7]`
- `can recover`
- `ls`
- `print <path>`
- `tasks`
- `stats`
- `test`
- `sensors`

Poznamka k `move`: kdyz nezadas `pitch`, CLI zkusi automaticky najit nejblizsi validni pitch pro IK.

---

## Wi-Fi a web

SoftAP default (pokud v NVS neni ulozena jina konfigurace):

- SSID: `RoboControl_<node_id>` (napr. `RoboControl_2`)
- heslo: `Robo-Control123`
- mDNS: `http://robo-control.local/`
- fallback IP: obvykle `http://192.168.4.1/`

HTTP endpointy:

- `GET /` -> `spage.html`
- `GET /settings` -> `settings.html`
- `GET /web/style.css`
- `GET /web/app.js`
- `GET /web/robocontrol.ico`
- `GET /favicon.ico`
- `POST /wifi_reset`
- `POST /wifi_config`
- `GET /api/limits`
- `GET /files`
- `GET|PUT|DELETE /file/<name>`
- `WS /ws`

---

## WebSocket API

### Prikazy klient -> server

- `{"joint":1,"angle":90}`
- `{"cmd":"sensors"}`
- `{"cmd":"move_xyz","x":10,"y":20,"z":30,"pitch":0}`
- `{"cmd":"home"}`
- `{"cmd":"arm"}`
- `{"cmd":"disarm"}`
- `{"cmd":"set_work_offset","x":0,"y":0,"z":0}`
- `{"cmd":"gcode_line","line":"G1 X10 Y5"}`
- `{"cmd":"run_gcode","filename":"test.gcode"}`
- `{"cmd":"gcode_stop"}`

### Telemetrie server -> klient

Broadcast perioda je cca `200 ms`.

```json
{
  "state": "READY",
  "can_node_id": 2,
  "armed": true,
  "referenced": true,
  "tcp_est_valid": true,
  "work_offset": { "x": 106.0, "y": 0.0, "z": 114.3 },
  "tcp_work": { "x": 0.0, "y": 0.0, "z": 0.0, "pitch": 20.0 },
  "sensors": [{ "id": 0, "angle": 75.0 }]
}
```

---

## CAN/TWAI protokol

Default adresace (11-bit standard ID):

- command ID: `CAN_CMD_ID_BASE + node` -> `0x100 + node`
- broadcast command ID: `0x17F`
- response ID: `CAN_RESP_ID_BASE + node` -> `0x180 + node`
- status/heartbeat ID: `CAN_STATUS_ID_BASE + node` -> `0x700 + node`

Podporovane command kody:

- `0x01 GET_STATUS`
- `0x02 ARM`
- `0x03 DISARM`
- `0x04 HOME`
- `0x05 STOP`
- `0x10 UPLOAD_BEGIN` (slot + size)
- `0x11 UPLOAD_DATA` (seq + payload)
- `0x12 UPLOAD_END`
- `0x13 PROGRAM_RUN`
- `0x14 PROGRAM_DELETE`
- `0x20 PREPARE`
- `0x21 SYNC_START`

Program sloty:

- pocet slotu: `CAN_PROGRAM_SLOT_COUNT = 4`
- ulozeni: `/spiffs/data/can_slot_<slot>.gcode`
- status frame perioda: `CAN_STATUS_PERIOD_MS = 500`

---

## G-code (podporovana podmnozina)

Parser umi:

- `G0`, `G1`
- `G4` (`P` v ms nebo `S` v sekundach)
- `G90`, `G91`
- `G20`, `G21`
- `M2`, `M30` (stop programu)
- `M10` (gripper open), `M11` (gripper close), `M280 S<deg>` (gripper angle)
- slova `F`, `X`, `Y`, `Z`, `P`, `S`
- komentare za `;`

Dulezite:

- G-code bezi ve WORK souradnicich.
- Pred spustenim musi byt robot referencovany (`home`) a musi mit znamou TCP pozici.
- Pri chybe dosahu, timeoutu nebo parser error se fronta flushne a beh se zastavi.

---

## File manager (SPIFFS)

- datovy adresar: `/spiffs/data`
- `GET /files` vraci seznam (`name`, `size`)
- `GET /file/<name>` vraci obsah
- `PUT /file/<name>` ulozi soubor (`.txt` nebo `.gcode`)
- `DELETE /file/<name>` smaze soubor

---

## Build a flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## Wokwi

Projekt obsahuje:

- `diagram.json`
- `wokwi.toml`

V simulaci je pouzita deska `board-esp32-s3-devkitc-1` se stejnym pin mapovanim jako v `config.h`.

---

## Aktualni omezeni

- Default build je sensorless (`SENSOR_COUNT=0`), telemetrie proto pouziva odhadnute uhly.
- WebSocket `move_xyz` nepouziva auto pitch scan (ten je pouze v CLI prikazu `move`).
- CAN je v defaultu v NO_ACK rezimu (`CAN_NO_ACK_MODE=1`) vhodnem hlavne pro bring-up/lab setup.

---

<p align="center">
  <img src="img/Web_interface.png" alt="Web UI" width="700" />
</p>

<p align="center">
  <img src="img/PCB_realization.png" alt="PCB realization" width="700" />
</p>
