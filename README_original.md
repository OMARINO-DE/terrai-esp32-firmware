# FarmOS ESP32 Firmware

PlatformIO-based firmware for ESP32 smart greenhouse controller.

## Quick Start

```bash
# Install PlatformIO
pip install platformio

# Create config
cp src/config.h.example src/config.h

# Edit config.h with your settings

# Build
pio run

# Upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor
```

## Hardware Support

### Sensors
- **DHT22** (default): Digital temperature/humidity sensor
- **SHT31**: I2C high-precision temperature/humidity sensor

Select sensor type in `platformio.ini`:
```ini
build_flags = -DSENSOR_DHT22
# OR
build_flags = -DSENSOR_SHT31
```

### Pinout
- GPIO 4: DHT22 data pin
- GPIO 21/22: SHT31 I2C (SDA/SCL)
- GPIO 25: Fan relay control

## Configuration

Edit `src/config.h`:

```cpp
// WiFi
#define WIFI_SSID "YourWiFi"
#define WIFI_PASS "YourPassword"

// MQTT
#define MQTT_HOST "your-server-ip"
#define MQTT_PORT 1883
#define MQTT_USER "pocuser"
#define MQTT_PASS "pocpass123"

// Device
#define DEVICE_ID "gh-001"

// GPIO
#define FAN_PIN 25
#define DHT_PIN 4
```

## Features

- Automatic WiFi reconnection
- MQTT telemetry every 30 seconds
- Hysteresis-based fan control
- Manual fan override
- Threshold persistence (NVS)
- Robust error handling

## Serial Monitor

Expected output:
```
FarmOS ESP32 Firmware
Version: esp32-poc-0.1.0
Device ID: gh-001

WiFi connected!
IP address: 192.168.1.25

MQTT connected!
Subscribed to command topics

Telemetry published:
{"ts":1730803200000,"temperature_c":27.4,"humidity_pct":61.2,"fan":false}
```

## Troubleshooting

- **WiFi fails**: Check SSID/password, use 2.4GHz only
- **MQTT fails**: Verify broker IP, credentials, firewall
- **Sensor NaN**: Check wiring, add pull-up resistor for DHT22
- **Fan not working**: Check GPIO wiring, relay voltage

See [docs/FIRMWARE.md](../../docs/FIRMWARE.md) for detailed guide.
