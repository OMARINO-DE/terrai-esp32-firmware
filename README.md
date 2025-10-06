# TerrAI ESP32 Firmware# FarmOS ESP32 Firmware



<div align="center">PlatformIO-based firmware for ESP32 smart greenhouse controller.



**🌱 Smart growth. Intelligent earth. 🌍**## Quick Start



[![PlatformIO](https://img.shields.io/badge/PlatformIO-Framework-orange.svg)](https://platformio.org)```bash

[![ESP32](https://img.shields.io/badge/ESP32-Supported-blue.svg)](https://www.espressif.com/en/products/socs/esp32)# Install PlatformIO

[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)pip install platformio



*by OMARINO IT Services*# Create config

cp src/config.h.example src/config.h

</div>

# Edit config.h with your settings

---

# Build

## 📋 Overviewpio run



TerrAI ESP32 Firmware is a robust, autonomous IoT controller for smart greenhouse and agricultural monitoring systems. Built on PlatformIO and designed for the ESP32 microcontroller, it provides:# Upload to ESP32

pio run -t upload

- **🌡️ Environmental Monitoring** - Temperature & humidity sensing

- **💨 Intelligent Fan Control** - Hysteresis-based automation with manual override# Monitor serial output

- **🔌 Autonomous Operation** - Works offline with saved thresholdspio device monitor

- **📡 MQTT/MQTTS Integration** - Cloud connectivity with TLS encryption support```

- **💾 Persistent Configuration** - NVS storage for thresholds

- **📺 LCD Display** - Optional I2C LCD for real-time status## Hardware Support



## ✨ Key Features### Sensors

- **DHT22** (default): Digital temperature/humidity sensor

### Autonomous Edge Intelligence- **SHT31**: I2C high-precision temperature/humidity sensor

- Operates independently without cloud connectivity

- Sensor readings and fan control work **100% offline**Select sensor type in `platformio.ini`:

- Automatically saves and loads thresholds from flash memory```ini

- Non-blocking reconnection attempts for WiFi and MQTTbuild_flags = -DSENSOR_DHT22

# OR

### Sensor Supportbuild_flags = -DSENSOR_SHT31

- **DHT11/DHT22** - Digital temperature/humidity sensors```

- **SHT31** - High-precision I2C temperature/humidity sensor

- Easy sensor selection via build flags### Pinout

- GPIO 4: DHT22 data pin

### Fan Control- GPIO 21/22: SHT31 I2C (SDA/SCL)

- **Automatic Mode** - Hysteresis-based control prevents oscillation- GPIO 25: Fan relay control

- **Manual Override** - Remote fan control via MQTT

- Configurable temperature and humidity thresholds## Configuration

- Persistent threshold storage across power cycles

Edit `src/config.h`:

### Connectivity

- **MQTT** - Standard unencrypted MQTT for local networks```cpp

- **MQTTS** - TLS-encrypted MQTT for secure internet connections// WiFi

- Automatic reconnection with exponential backoff#define WIFI_SSID "YourWiFi"

- WiFi signal strength (RSSI) reporting#define WIFI_PASS "YourPassword"



### Optional LCD Display// MQTT

- 16x2 or 20x4 I2C LCD support#define MQTT_HOST "your-server-ip"

- Real-time temperature, humidity, fan status#define MQTT_PORT 1883

- Connection status indicators (ONLINE/OFFLINE/WiFi)#define MQTT_USER "pocuser"

#define MQTT_PASS "pocpass123"

---

// Device

## 🚀 Quick Start#define DEVICE_ID "gh-001"



### Prerequisites// GPIO

#define FAN_PIN 25

- ESP32 development board#define DHT_PIN 4

- Temperature/Humidity sensor (DHT11, DHT22, or SHT31)```

- Relay module or MOSFET for fan control

- PlatformIO IDE or CLI## Features



### Installation- Automatic WiFi reconnection

- MQTT telemetry every 30 seconds

1. **Clone the repository**- Hysteresis-based fan control

```bash- Manual fan override

git clone https://github.com/OMARINO-DE/terrai-esp32-firmware.git- Threshold persistence (NVS)

cd terrai-esp32-firmware- Robust error handling

```

## Serial Monitor

2. **Install PlatformIO** (if not already installed)

```bashExpected output:

pip install platformio```

```FarmOS ESP32 Firmware

Version: esp32-poc-0.1.0

3. **Create configuration file**Device ID: gh-001

```bash

cp src/config.h.example src/config.hWiFi connected!

```IP address: 192.168.1.25



4. **Edit `src/config.h` with your settings**MQTT connected!

```cppSubscribed to command topics

#define WIFI_SSID "YourWiFi"

#define WIFI_PASS "YourPassword"Telemetry published:

#define MQTT_HOST "your-broker.example.com"{"ts":1730803200000,"temperature_c":27.4,"humidity_pct":61.2,"fan":false}

#define MQTT_PORT 1883```

#define DEVICE_ID "gh-001"

```## Troubleshooting



5. **Build and upload**- **WiFi fails**: Check SSID/password, use 2.4GHz only

```bash- **MQTT fails**: Verify broker IP, credentials, firewall

# Build firmware- **Sensor NaN**: Check wiring, add pull-up resistor for DHT22

pio run- **Fan not working**: Check GPIO wiring, relay voltage



# Upload to ESP32See [docs/FIRMWARE.md](../../docs/FIRMWARE.md) for detailed guide.

pio run -t upload

# Monitor serial output
pio device monitor
```

---

## 🔧 Hardware Setup

### GPIO Pinout

| Component | GPIO Pin | Description |
|-----------|----------|-------------|
| DHT11/DHT22 | GPIO 4 | Temperature/Humidity sensor data pin |
| SHT31 SDA | GPIO 21 | I2C data line for SHT31 |
| SHT31 SCL | GPIO 22 | I2C clock line for SHT31 |
| Fan Control | GPIO 25 | Relay/MOSFET control for fan |
| LCD SDA | GPIO 21 | I2C data for LCD (shared with SHT31) |
| LCD SCL | GPIO 22 | I2C clock for LCD (shared with SHT31) |

### Wiring Examples

#### DHT22 Wiring
```
DHT22          ESP32
  VCC    ->    3.3V
  DATA   ->    GPIO 4
  GND    ->    GND
```
*Note: Add 10kΩ pull-up resistor between DATA and VCC*

#### SHT31 Wiring
```
SHT31          ESP32
  VCC    ->    3.3V
  SDA    ->    GPIO 21
  SCL    ->    GPIO 22
  GND    ->    GND
```

#### Fan Control Wiring
```
Relay Module   ESP32
  VCC    ->    5V
  IN     ->    GPIO 25
  GND    ->    GND
```

---

## ⚙️ Configuration

### Sensor Selection

Edit `platformio.ini` to select your sensor:

```ini
build_flags = 
    -DSENSOR_DHT11      ; For DHT11
    ; -DSENSOR_DHT22    ; For DHT22
    ; -DSENSOR_SHT31    ; For SHT31
```

### Enable LCD Display

```ini
build_flags = 
    -DSENSOR_DHT11
    -DUSE_LCD_I2C       ; Enable LCD display
```

### MQTTS (TLS Encryption)

For secure internet connectivity:

1. Enable TLS in `src/config.h`:
```cpp
#define MQTT_USE_TLS 1
#define MQTT_PORT 8883
```

2. Add certificates to `src/certificates.h`:
```cpp
const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"Your root CA certificate here\n" \
"-----END CERTIFICATE-----\n";
```

### Default Thresholds

Customize in `src/config.h`:
```cpp
#define DEFAULT_TEMP_ON 28.0    // Fan turns ON above 28°C
#define DEFAULT_TEMP_OFF 26.0   // Fan turns OFF below 26°C
#define DEFAULT_RH_ON 80.0      // Fan turns ON above 80% humidity
#define DEFAULT_RH_OFF 75.0     // Fan turns OFF below 75% humidity
```

---

## 📡 MQTT Topics

The firmware uses the following topic structure:

### Published Topics
- `terrai/{DEVICE_ID}/telemetry` - Periodic sensor readings
- `terrai/{DEVICE_ID}/state` - Device state and metadata
- `terrai/{DEVICE_ID}/ack` - Command acknowledgments

### Subscribed Topics
- `terrai/{DEVICE_ID}/cmd/thresholds` - Update temperature/humidity thresholds
- `terrai/{DEVICE_ID}/cmd/fan` - Manual fan control

### Example Messages

**Telemetry:**
```json
{
  "ts": 1730803200000,
  "temperature_c": 27.4,
  "humidity_pct": 61.2,
  "fan": false,
  "rssi": -42,
  "fw": "esp32-poc-0.1.0"
}
```

**Threshold Command:**
```json
{
  "temp_on": 28.0,
  "temp_off": 26.0,
  "rh_on": 80.0,
  "rh_off": 75.0
}
```

**Fan Command:**
```json
{
  "mode": "manual",
  "state": true
}
```
or
```json
{
  "mode": "auto"
}
```

---

## 🖥️ Serial Monitor Output

Successful startup looks like:
```
╔══════════════════════════════════════════════════════════╗
║         TerrAI ESP32 Firmware v1.1.0                    ║
║         Smart growth. Intelligent earth.                ║
║         by OMARINO IT Services                           ║
╚══════════════════════════════════════════════════════════╝

Device ID: gh-001
Firmware: esp32-poc-0.1.0
Mode: MQTT (Local)

Initializing DHT11 sensor...
Loaded thresholds from NVS:
  temp_on=28.0, temp_off=26.0, rh_on=80.0, rh_off=75.0

Connecting to WiFi: YourWiFi
..........
WiFi connected!
IP address: 192.168.1.25
RSSI: -42

🔌 Connecting to MQTT broker: your-broker.example.com:1883
✅ MQTT connected!
   Subscribed to command topics

=================================
Setup complete!
=================================
Mode: AUTONOMOUS (edge intelligence)
- Sensors: Reading locally
- Fan control: Using saved thresholds
- Network: Will retry in background
=================================

Starting main loop...
```

---

## 🛠️ Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| WiFi connection fails | Wrong SSID/password | Verify credentials, use 2.4GHz WiFi only |
| MQTT connection fails | Wrong broker IP/credentials | Check `MQTT_HOST`, `MQTT_USER`, `MQTT_PASS` |
| Sensor returns NaN | Wiring issue | Check connections, add pull-up resistor for DHT |
| Fan doesn't activate | GPIO issue | Verify relay wiring, check power supply |
| Compilation errors | Missing libraries | Run `pio lib install` to fetch dependencies |

---

## 📚 Development

### Build Environments

The firmware supports multiple build configurations:

```bash
# Build for DHT11 (default)
pio run -e esp32dev

# Build for SHT31
pio run -e esp32dev-sht31
```

### Dependencies

Automatically managed by PlatformIO:
- ArduinoJson (^6.21.3)
- PubSubClient (^2.8)
- DHT sensor library (^1.4.4)
- Adafruit Unified Sensor (^1.1.9)
- LiquidCrystal_I2C (^1.1.4)

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## 📧 Support

For issues, questions, or feature requests:
- Open an issue on GitHub
- Contact: OMARINO IT Services

---

## 🌟 Acknowledgments

Built with ❤️ using:
- [PlatformIO](https://platformio.org/)
- [ESP32](https://www.espressif.com/en/products/socs/esp32)
- [Arduino Framework](https://www.arduino.cc/)

---

<div align="center">

**TerrAI** - Smart growth. Intelligent earth.

*Empowering sustainable agriculture through IoT*

</div>
