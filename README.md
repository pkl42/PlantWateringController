# PlantWateringController

An open-source IoT system for **automated plant watering and monitoring**, built around an **ESP32 controller**, **Tasmota ESP8266 sensors**, and a **Node-RED dashboard** with **InfluxDB** time-series storage.

---

## Features
- ESP32-based **Plant Controller** for moisture-driven watering control  
- ESP8266 Wi-Fi sensors (Tasmota) with **deep sleep** and MQTT telemetry  
- **Node-RED Dashboard 2.0** for live visualization and manual override  
- **InfluxDB 2.0** for time-series logging and analytics  
- Modular 3D-printed housings (Fusion 360) for pumps, sensors, and enclosures  
- Supports both **wired** and **wireless** capacitive moisture sensors

---

## System Overview

[ ESP8266 Sensor ] ─── MQTT ───┐
│
[ Node-RED ] ─── [ InfluxDB ]
│
[ ESP32 Controller ] <─────────┘


---

## Quick Start

### 1️⃣ Flash the ESP32 Controller
- Code in: `/firmware/esp32_controller`
- Built using **PlatformIO**
- Configure Wi-Fi and MQTT broker in `config.h`

### 2️⃣ Configure ESP8266 Sensor
- Flash **Tasmota**
- Apply rules from `/firmware/esp8266_sensor_tasmota/tasmota_rules.md`
- Set up **telemetry** every few minutes
- Example payload: `tele/plantSensor/STATE`

### 3️⃣ Import Node-RED Flow
- Import `/nodered/flows.json`
- Set MQTT and InfluxDB connection nodes
- Adjust dashboard layout if needed

### 4️⃣ InfluxDB Setup
- Create a bucket, retention policy, and token
- See `/influxdb/retention_policy_setup.md`

### 5️⃣ Optional Hardware Setup
- Print 3D housings from `/cad/`
- See `/docs/build_guide.md` and `/docs/wiring.md`

---

## Repository Structure
| Folder | Description |
|---------|--------------|
| `firmware/` | ESP32 controller firmware + Tasmota sensor setup |
| `nodered/` | Dashboard flow and UI elements |
| `cad/` | Fusion 360 source, STL exports, and renders |
| `hardware/` | Wiring diagrams and bill of materials |
| `influxdb/` | Database configuration and example queries |
| `docs/` | Guides, build notes, and overviews |

---

## Recommended Tools
- [PlatformIO](https://platformio.org/)
- [Node-RED Dashboard 2.0](https://flows.nodered.org/)
- [InfluxDB 2.x](https://www.influxdata.com/)
- [Fusion 360](https://www.autodesk.com/products/fusion-360/)
- [Fritzing](https://fritzing.org/) (optional for wiring diagrams)

---

## License
This project is open-source.  
You can use or modify it under the **Apache 2.0 License** (see [LICENSE](LICENSE)).

---

## Author
Created by **Peter Ludwig**  
Designed for modular smart plant care and automation.
