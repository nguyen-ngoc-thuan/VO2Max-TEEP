# VO2Max-TEEP

**Improved DIY portable spirometer for measuring VO2 Max**

An ESP32-based wearable that measures oxygen consumption (VO2) during exercise, designed for use with Zwift, Strava, and Golden Cheetah.

## Features

- **Modular FreeRTOS architecture** — sensor reading runs at highest priority, uninterrupted by WiFi/BLE/display
- **Real-time VO2 Max calculation** with configurable integration time (5/10/15/30/60s)
- **Multiple connectivity options**: WiFi (UDP streaming), BLE HR profile (Zwift/Strava), Golden Cheetah, Bluetooth Serial
- **On-device data storage** with ring buffer (1024 samples) and flash file logging
- **Configurable Venturi diameter** (16/19/20mm) from menu — no recompile needed
- **PC companion tool** (Python/PyQtGraph) for real-time visualization and CSV recording
- **5 display screens**: VO2+HR, detailed metrics, breathing, environment, sensor status

## Hardware

| Component | Model | I2C Address |
|---|---|---|
| Microcontroller | TTGO T-Display (ESP32) | — |
| Differential Pressure | Omron D6F-PH0025AMD2 | 0x6C |
| Oxygen Sensor | DFRobot SEN0322 (Gravity I2C) | 0x73 |
| Ambient Pressure/Temp | BMP280 | 0x76 |
| Display | Built-in 1.14" TFT (ST7789) | — |
| Battery | LiPo 3.7V 1000mAh | — |

## Wiring

All sensors use I2C bus (SDA=Pin21, SCL=Pin22):

```
ESP32 Pin 21 (SDA) → all sensor SDA
ESP32 Pin 22 (SCL) → all sensor SCL
ESP32 3V3           → all sensor VCC
ESP32 GND           → all sensor GND
```

## Building

### Prerequisites
- [VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)

### Build & Upload
```bash
pio run                    # Build
pio run --target upload    # Upload to board
pio device monitor         # Serial monitor
```

## Usage

1. Power on → sensor initialization (3s)
2. Press upper button for **Settings Menu** (or wait to skip)
3. **O2 warm-up** countdown (3 min, press any button to skip)
4. **Ready!** — put on mask and exercise
5. Navigate screens with upper/lower buttons
6. Hold both buttons 2s to restart

## Menu Options

| Option | Description |
|---|---|
| Set weight | Body weight in kg (for ml/min/kg) |
| Interval | Integration time (5s to 60s) |
| Venturi D | Inner diameter (16/19/20mm) |
| Cal. O2 | Recalibrate O2 sensor (20.9%) |
| Cal. Flow | Flow calibration with 3L syringe |
| WiFi | Enable WiFi AP for PC streaming |
| HR belt | Connect to BLE heart rate monitor |
| HR out | Broadcast VO2 as BPM (for Zwift) |
| Cheetah | Enable Golden Cheetah VO2 Master |

## PC Monitor Tool

```bash
cd tools
pip install pyqtgraph numpy PyQt5
python vo2max_monitor.py
```

Connect to the VO2MAX WiFi AP first, then run the script.

## 3D Print Files

Use 3D print files from the original V1 design:
- VenturiSensor — Venturi tube + sensor mount
- MainBody — Main enclosure
- SideComputer — T-Display housing
- MaskAttachment — 3M 6200 mask adapter

## Credits

Based on:
- [Accurate VO2 Max for Zwift and Strava](https://www.instructables.com/Accurate-VO2-Max-for-Zwift-and-Strava/) by Rabbitcreek
- [VO2max-main](https://github.com/meteoscientific/VO2max) by meteoscientific / Ivor Hewitt
- [VO2Max-500pa](https://github.com/zanppa/VO2Max) by Lauri Peltonen

## License

GPL V3
