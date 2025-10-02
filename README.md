# Ovládání modelu manipulátoru pomocí ESP32-S3

Tento projekt je ukázka řízení 6DOF robotického ramene pomocí **ESP32-S3**.  
Implementuje ovládání servomotorů, čtení senzorů a základní inverzní kinematiku.  
Komunikace probíhá přes **WebSocket/HTTP server** a **UART příkazy**.

---

## 📂 Struktura projektu
```
main/
├── main.c # Vstupní bod (app_main)
├── robot_io/ # Serva a senzory
│ ├── robot_io.c
│ └── robot_io.h
├── uart_receive/ # UART příkazy
│ ├── uart_receive.c
│ └── uart_receive.h
├── wifi_server/ # HTTP + WebSocket server
│ ├── wifi_server.c
│ └── wifi_server.h
└── CMakeLists.txt
```

---

## 🔌 Hardware

- **ESP32-S3 DevKitC**
- 6× servo motor (GPIO 35–41)
- senzory (ADC kanály)
- volitelně USB-TTL převodník (CP2102/CH340) pro UART

### 🔹 Serva (PWM přes LEDC)

| Servo ID | GPIO pin | LEDC Channel |
|----------|----------|--------------|
| 0        | 35       | 0            |
| 1        | 36       | 1            |
| 2        | 37       | 2            |
| 3        | 39       | 3            |
| 4        | 40       | 4            |
| 5        | 41       | 5            |
| (6)      | 42       | 6 *(rezervace pro manipulátor)* |

---

### 🔹 Senzory (ADC)

| Sensor ID | ADC Unit | ADC Channel | GPIO pin |
|-----------|----------|-------------|----------|
| 0         | 1        | 3           | IO4      |
| 1         | 1        | 4           | IO5      |
| 2         | 1        | 5           | IO6      |
| 3         | 1        | 6           | IO7      |
| 4         | 1        | 7           | IO12     |
| 5         | 2        | 6           | IO17     |
| (6)       | 2        | 7           | IO18 *(rezervace)* |

---

## ⚙️ Funkce

### UART příkazy

| Příkaz         | Popis                                      | Příklad         |
|----------------|--------------------------------------------|-----------------|
| `SERVO id ang` | Nastaví servo na zadaný úhel (0–180°)      | `SERVO 0 90`    |
| `MOVE x y z`   | Pohne ramenem do souřadnic (IK triangulace)| `MOVE 10 20 30` |
| `SENSORS?`     | Vrátí úhly ze všech senzorů                | `SENSORS?`      |

---

### Web server (HTTP/WS)

- `/` – hlavní stránka (HTML)  
- `/ws` – WebSocket API pro real-time komunikaci  
- `/upload` – nahrání G-code souboru  
- `/status` – JSON se stavem připojení  
- `/settings` – nastavení Wi-Fi  
- `/wifi_reset` – reset Wi-Fi konfigurace  

---

## 🚀 Build a flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
(nahraď COMx portem svého ESP32-S3)

🛠️ TODO
vylepšená inverzní kinematika (6DOF)

interpolace dráhy (trajektorie místo point-to-point)

webová vizualizace polohy ramen

yaml
Zkopírovat kód

---

Chceš, abych ti tam ještě dopsal i ukázku, jak posílat příkazy přes **Python script (pyserial)** pro 
