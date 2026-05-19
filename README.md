# RoboControl

Firmware pro rizeni 5DOF robotickeho ramene zapomocí ESP32-S3.

Aktualni implementace obsahuje:

- rizeni 6 kloubu pres 7 PWM serv (J1 ma follower servo, J5 je gripper),
- sensorless rezim s odhadovanou polohou kloubu (`SENSOR_COUNT = 0`),
- HTTP Web UI, WebSocket telemetrii a SPIFFS file manager,
- UART konzoli pres `esp_console`,
- G-code frontu a vykonavani programu ze SPIFFS,
- CAN/TWAI protokol pro rizeni uzlu a synchronizovane spousteni programu,
- stavovou WS2812 LED a zapis runtime statistik do SPIFFS.

## Aktualni profil

Konfigurace je v `main/config.h`.

| Volba | Hodnota |
| --- | --- |
| Target | `esp32s3` |
| Flash | `8MB` |
| Robot node | `ROBOT_NODE_ID = 1` |
| CAN node | `CAN_NODE_ID = 1` |
| CAN bitrate | `500000` |
| CAN mode | `NO_ACK` (`CAN_NO_ACK_MODE = 1`) |
| SoftAP SSID | `RoboControl_1` |
| SoftAP heslo | `Robo-Control123` |
| mDNS | `http://robo-control.local/` |
| SPIFFS data | `/spiffs/data` |
| SPIFFS web | `/spiffs/web` |

## Build a flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

SPIFFS image se vytvari z adresare `spiffs/` a je soucasti flashovani (`FLASH_IN_PROJECT`).

## Struktura

```text
main/
  main.c
  config.h
  robot_io/          servo vystupy, IK, planner, stavy robota
  wifi_server/       HTTP, WebSocket, file manager
  cmd_control/       UART REPL prikazy
  gcode/             parser a vykonavani G-code
  can_communication/ CAN/TWAI protokol
  status_led/        onboard WS2812 stavova LED
  rt_stats/          runtime statistiky
spiffs/
  web/               webove UI
  data/              .gcode/.txt soubory
```

## Pinout

| Funkce | GPIO |
| --- | --- |
| J0 | 35 |
| J1 master | 36 |
| J1 follower | 37 |
| J2 | 39 |
| J3 | 40 |
| J4 | 41 |
| J5 / gripper | 42 |
| CAN TX | 14 |
| CAN RX | 13 |
| Onboard WS2812 | 38 |
| Sync measure | 3 |

## Ovladani

Po startu firmware inicializuje SPIFFS, NVS, serva, SoftAP web server, robot control task, CAN/TWAI, stavovou LED a UART REPL.

Zakladni UART prikazy:

```text
arm
home
state
move <x> <y> <z> [pitch]
joint <id> <angle>
gcode run <file.gcode>
gcode line <G-code>
gcode stop
gcode reset
gcode sync
wcofs [x y z]
ls
print <file>
can status
can probe [count] [gap_ms]
can tx <id> [b0 ... b7]
disarm
```

Pohybove prikazy pracuji ve WORK souradnicich. `home` nastavi referenci az po dokonceni HOME sekvence.

## Web a API

- UI: `/`
- nastaveni Wi-Fi: `/settings`
- WebSocket: `/ws`
- limity kloubu: `GET /api/limits`
- seznam souboru: `GET /files`
- soubor: `GET|PUT|DELETE /file/<name>`

WebSocket periodicky posila stav robota, CAN node, referenci, WORK offset, odhad TCP a odhad kloubu. Prijima napr. `joint`, `move_xyz`, `home`, `arm`, `disarm`, `set_work_offset`, `gcode_line`, `run_gcode` a `gcode_stop`.

## CAN/TWAI

Standardni 11-bit ID:

- command: `0x100 + node`
- broadcast command: `0x17F`
- response: `0x180 + node`
- status: `0x700 + node`
- info: `0x780 + node`

Podporovane akce: status, arm, disarm, home, stop, upload programu do slotu, spusteni/smazani slotu, prepare a sync start. Sloty se ukladaji jako `/spiffs/data/can_slot_<slot>.gcode`.

## G-code

Podmnozina parseru:

- `G0`, `G1`, `G4`
- `G90`, `G91`
- `G20`, `G21`
- `M2`, `M30`
- `M10`, `M11`, `M280 S<deg>`

Pohybovy G-code vyzaduje armed robota, referenci a znamy odhad TCP. Gripper-only program lze spustit i bez reference.

## Poznamky

- ADC senzory jsou v aktualnim buildu vypnute (`SENSOR_COUNT = 0`).
- WebSocket `move_xyz` pouziva zadany/default pitch; auto pitch scan je jen v UART prikazu `move`.
- Runtime statistiky se zapisuji do `/spiffs/data/rt_stats.txt`.
- Stavy robota: `DISARMED`, `UNREFERENCED`, `READY`, `RUNNING`, `READY_FOR_SYNC`, `ERROR`.
