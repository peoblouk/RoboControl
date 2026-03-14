<!-- @format -->

# RoboControl

<p align="right">
  <img src="spiffs/web/robocontrol.ico" alt="RoboControl icon" width="50" />
</p>

RoboControl je projekt pro rizeni 6DOF robotickeho ramene na ESP32-S3.
Aktualni verze pouziva:

- Web UI + WebSocket (`/ws`)
- UART konzoli pres `esp_console` REPL (linenoise)
- planner s frontou pohybu, arm/disarm bezpecnost a HOME/reference workflow
- stavovou onboard RGB LED (WS2812)

---

## Co je hotove

- Rizeni 6 kloubu (+ follower servo pro J1 a rezerva pro gripper)
- Jednoducha planarni IK (`move_xyz`) s work offsetem
- HOME prikaz, ktery zalozi referenci robota
- G-code pipeline (`run file`, `line`, `stop`, `reset`, `sync`)
- SoftAP HTTP server + WebSocket telemetrie
- File manager pro `.txt` a `.gcode` ve SPIFFS
- Stavova RGB LED na desce ESP32-S3-DevKitC-1

---

## Struktura projektu

```text
main/
|-- main.c
|-- config.h
|-- robot_io/
|   |-- robot_io.c
|   `-- robot_io.h
|-- wifi_server/
|   |-- wifi_server.c
|   `-- wifi_server.h
|-- cmd_control/
|   |-- cmd_control.c
|   `-- cmd_control.h
|-- gcode/
|   |-- gcode.c
|   `-- gcode.h
|-- status_led/
|   |-- status_led.c
|   `-- status_led.h
`-- rt_stats/
    |-- rt_stats.c
    `-- rt_stats.h
```

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

### Senzory (ADC map v konfiguraci)

| Sensor ID | ADC unit | ADC channel | GPIO |
| --- | --- | --- | --- |
| 0 | 1 | 3 | IO4 |
| 1 | 1 | 4 | IO5 |
| 2 | 1 | 5 | IO6 |
| 3 | 1 | 6 | IO7 |
| 4 | 1 | 7 | IO12 |
| 5 | 2 | 6 | IO17 |

Poznamka:
- V `main.c` je `sensors_init()` aktualne vypnute.
- Web telemetrie proto pouziva odhadnute kloubove uhly (`robot_get_est_angle`), ne live ADC.

### Onboard RGB LED

- WS2812 data pin: `GPIO38` (`STATUS_LED_WS2812_GPIO`)
- Stavy LED:
- `DISARMED` = cervena
- `ARMED` = zelena
- `OPERATING` = oranzova blikani

---

## Robot stavy a bezpecnost

- `disarm` vypne PWM vystupy a zahodi frontu pohybu.
- `arm` znovu povoli rizeni pohybu.
- Prikazy `joint` a `move` jsou blokovane, kdyz je robot disarmed.
- `move` pracuje ve WORK souradnicich a vyzaduje referenci.
- Referenci nastavuje `home` (po dokonceni HOME segmentu).

Stav vraceny pres WS:
- `DISARMED`
- `OPERATING`
- `UNREFERENCED`
- `POSE_UNKNOWN`
- `IDLE`

---

## Aktualni omezeni

- `move <x y z [pitch]>` aktualne pouziva simple planarni IK; pitch je zatim metadata a neni plne resen v IK.
- `sensors_init()` je v `main.c` vypnute, takze system bezi defaultne v sensorless rezimu (web ukazuje odhadnute uhly).

---

## Konzole (UART REPL)

Konzole bezi pres `esp_console` REPL:

- prompt `>`
- historie prikazu: 5 polozek
- editace radku + sipky (linenoise)
- logy jsou tisknute tak, aby nerozbily rozepsany radek

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
- `ls`
- `print <path>`
- `tasks`
- `stats`
- `test`
- `sensors` (ADC; ma smysl az pri zapnutem `sensors_init()`)

---

## Wi-Fi a web

SoftAP default:

- SSID: `RoboControl`
- heslo: `Robo-Control123`
- mDNS: `http://robo-control.local/`
- fallback IP: typicky `http://192.168.4.1/` (default AP IP ESP-IDF)

HTTP endpointy:

- `GET /` -> hlavni stranka (`spage.html`)
- `GET /settings` -> settings stranka
- `GET /status` -> `{ "online": true/false }`
- `POST /wifi_reset`
- `GET|POST /wifi_config`
- `GET /api/limits`
- `POST /upload`
- `GET /files`
- `GET|PUT|DELETE /file/<name>`
- `WS /ws`

---

## WebSocket API

### Prikazy klient -> server

- `{"joint":1,"angle":90}`
- `{"servo":3,"angle":90}` (kompatibilni alias)
- `{"cmd":"sensors"}`
- `{"cmd":"move_xyz","x":10,"y":20,"z":30,"pitch":0}`
- `{"cmd":"home"}`
- `{"cmd":"arm"}`
- `{"cmd":"disarm"}`
- `{"cmd":"set_work_offset","x":0,"y":0,"z":0}`
- `{"cmd":"gcode_line","line":"G1 X10 Y5"}`
- `{"cmd":"run_gcode","filename":"test.gcode"}`
- `{"cmd":"gcode_stop"}`

### Telemetrie server -> klient (periodicky)

Posila se cca kazdych 200 ms:

```json
{
  "state": "IDLE",
  "armed": true,
  "referenced": true,
  "tcp_est_valid": true,
  "work_offset": { "x": 0, "y": 0, "z": 0 },
  "tcp_work": { "x": 0, "y": 0, "z": 0, "pitch": 0 },
  "sensors": [{ "id": 0, "angle": 75.0 }]
}
```

---

## G-code (aktualne podporovana podmnozina)

Parser umi:

- `G0`, `G1`
- `G90`, `G91`
- `G20`, `G21`
- `F`, `X`, `Y`, `Z`, `P`
- komentare za `;`

Dulezite:

- G-code bezi ve WORK souradnicich.
- Pred spustenim musi byt robot referencovany (`home`) a mit znamou TCP pozici.
- Pri chybe dosahu nebo timeoutu se fronta flushne a beh se zastavi.

---

## File manager (SPIFFS)

- Datovy adresar: `/spiffs/data`
- Povoleny upload/extenze: `.txt`, `.gcode`
- `GET /files` vraci seznam souboru a velikosti
- `GET /file/<name>` vraci obsah
- `PUT /file/<name>` ulozi soubor
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

V simulaci je pouzita deska `board-esp32-s3-devkitc-1` a stejne pin mapovani jako v `config.h`.

---

<p align="center">
  <img src="img/Web_interface.png" alt="Web UI" width="700" />
</p>

<p align="center">
  <img src="img/PCB_realization.png" alt="PCB realization" width="700" />
</p>
