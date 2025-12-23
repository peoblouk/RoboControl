<!-- @format -->

# Ovládání modelu manipulátoru pomocí ESP32-S3

<p align="right">
  <img src="spiffs/web/robocontrol.ico" alt="Web server interface" width="50"/>
</p>
Tento projekt je ukázka řízení 6DOF robotického ramene pomocí <b>ESP32-S3</b>.  
Implementuje ovládání servomotorů, čtení senzorů a základní inverzní kinematiku.  
Komunikace probíhá přes <b>WebSocket/HTTP server</b> a <b>UART/konzolové</b> příkazy.


---

## Struktura projektu

```
main/
├── main.c # Vstupní bod (app_main)
├── robot_io/ # Serva a senzory, IK a řízení
│ ├── robot_io.c
│ └── robot_io.h
├── wifi_server/ # HTTP + WebSocket server + file manager
│ ├── wifi_server.c
│ └── wifi_server.h
├── cmd_control/ # Konzolové příkazy (esp_console) a CLI
│ ├── cmd_control.c
│ └── cmd_control.h
├── rt_stats/ # Real-time statistiky (měření latencí)
│ ├── rt_stats.c
│ └── rt_stats.h
└── CMakeLists.txt
```

---

## Hardware

<p align="center">
  <img src="img/RobotControl_3D.png" alt="3D Render" width="700"/>
</p>

- **ESP32-S3 DevKitC**
- 6× servo motor (GPIO 35–41)
- senzory (ADC kanály)
- volitelně USB-TTL převodník (CP2102/CH340) pro UART/konzoli

### Serva (PWM přes LEDC)

| Servo ID | GPIO pin | LEDC Channel                    |
| -------- | -------- | ------------------------------- |
| 0        | 35       | 0                               |
| 1        | 36       | 1                               |
| 2        | 37       | 2                               |
| 3        | 39       | 3                               |
| 4        | 40       | 4                               |
| 5        | 41       | 5                               |
| (6)      | 42       | 6 _(rezervace pro manipulátor)_ |

---

### Senzory (ADC)

| Sensor ID | ADC Unit | ADC Channel | GPIO pin           |
| --------- | -------- | ----------- | ------------------ |
| 0         | 1        | 3           | IO4                |
| 1         | 1        | 4           | IO5                |
| 2         | 1        | 5           | IO6                |
| 3         | 1        | 6           | IO7                |
| 4         | 1        | 7           | IO12               |
| 5         | 2        | 6           | IO17               |
| (6)       | 2        | 7           | IO18 _(rezervace)_ |

---

## Funkce

### Inverzní kinematika (triangulační)

<p align="center">
  <img src="img/IK_calculation.png" alt="Inverse kinematic" width="700"/>
</p>

- Základní triangulační IK pro výpočet úhlů z (x,y,z).
- Podpora bezpečnostních limitů serv a interpolovaného pohybu (INTERP_STEPS).
- Move se provádí voláním robot_cmd_move_xyz / robot_cmd_move_joints, které queue-ují příkazy do robot_control_task.

### cmd_control (konzole / UART)
- Modul používá esp_console (console_task) a registruje příkazy dostupné přes UART/terminál.
- Hlavní příkazy:
  - servo <id> <angle> — nastaví servo
  - move <x> <y> <z> — enqueue MOVE XYZ
  - sensors — vypíše úhly všech senzorů
  - test — zařadí test sekvenci pohybu
  - print <filename> — vytiskne obsah souboru z FILE_STORAGE_PATH (/spiffs/data)
  - ls — list souborů v /spiffs/data
  - stats — vytiskne shromážděné měřicí statistiky (rt_stats)
  - tasks — vypíše seznam FreeRTOS úloh
- Konzole běží jako task připnutý na CORE_ROBOT.

### rt_stats (měření latencí)
- rt_stats sleduje dobu vykonání příkazů (servo, sensors, move) pomocí esp_timer_get_time().
- Uchovává count, min, max, mean a M2 pro výpočet směrodatné odchylky.
- cmd_control jej poté používá pro periodické tisknutí statistik (např. každých N vzorků).

### File manager a G-code
- Soubory jsou uloženy pod /spiffs/data (FM_BASE).
- Povolené přípony: .txt, .gcode
- HTTP endpoints pro správu souborů:
  - GET /files — seznam souborů (JSON)
  - GET /file/<name> — stáhne soubor (pouze .txt/.gcode)
  - PUT /file/<name> — uloží obsah do /spiffs/data/<name>
  - DELETE /file/<name> — odstraní soubor
  - POST /upload — ukládá nahraný G-code jako /spiffs/data/gcode_file.gcode (aktuální implementace)
- upload a čtení jsou obsluhovány ve wifi_server.c (chunked read/write).

### Web server (HTTP/WS)
- `/` – hlavní stránka (HTML)
- `/ws` – WebSocket API pro real-time komunikaci
  - Server periodicky broadcastuje JSON s poli "sensors" pro všechny klienty (interval ~200 ms, WS_SENSORS_PERIOD_MS).
  - Přijímá JSON příkazy od klienta:
    - {"servo": 0, "angle": 90}  — okamžité nastavení serva
    - {"cmd":"sensors"}          — okamžité vyslání hodnot senzorů klientovi
    - {"cmd":"move_xyz","x":10,"y":20,"z":30} — dá do fronty pohyb (vrací queued:true nebo error)
  - Po připojení server posílá {"status":"connected"}.
- `/status` — JSON stav (např. online true/false)
- `/settings` — stránka nastavení Wi‑Fi
- `/wifi_reset` — POST pro reset Wi‑Fi konfigurace (vymaže NVS a restartuje)

<p align="center">
  <img src="img/Web_interface.JPEG" alt="Web server interface" width="700"/>
</p>

---

## Protokoly / formáty

- WebSocket sensor broadcast: {"sensors":[{"id":0,"angle":12.3},...]}
- WS příkaz servo: {"servo":<id>,"angle":<deg>}
- WS příkaz move: {"cmd":"move_xyz","x":<float>,"y":<float>,"z":<float>}
- HTTP /files returns: [{"name":"test.gcode","size":123},...]

---

| GENEROVANO AI |

