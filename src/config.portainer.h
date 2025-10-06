// TerrAI ESP32 Configuration - PORTAINER VERSION
// Hardware: ESP32 Dev Module
// Sensors: DHT11 on GPIO26
// Actuator: Fan on GPIO27
// Display: LiquidCrystal I2C (0x27 or 0x3F)
// Server: Portainer @ 192.168.61.21

#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// WiFi Configuration
// ============================================
#define WIFI_SSID "2.4G"        // <-- Your WiFi network name
#define WIFI_PASS "c@@lc@@lc@@lc@@l"    // <-- Your WiFi password

// ============================================
// MQTT Broker Configuration - PORTAINER SERVER
// ============================================
#define MQTT_HOST "192.168.61.21"  // TerrAI Portainer server IP
#define MQTT_PORT 1883
#define MQTT_USER "pocuser"
#define MQTT_PASS "pocpass123"

// ============================================
// Device Configuration
// ============================================
#define DEVICE_ID "gh-esp32"              // Unique device identifier
#define FIRMWARE_VERSION "terrai-esp32-1.0.0"

// ============================================
// GPIO Pin Configuration
// ============================================
#define FAN_PIN 27           // GPIO27 - Fan relay/MOSFET control
#define DHT_PIN 26           // GPIO26 - DHT11 sensor data pin

// LCD I2C Configuration
#define LCD_I2C_ADDR 0x27    // Try 0x3F if 0x27 doesn't work
#define LCD_COLS 16          // 16 columns
#define LCD_ROWS 2           // 2 rows

// I2C Pins (ESP32 default)
#define I2C_SDA 21
#define I2C_SCL 22

// ============================================
// Timing Configuration (milliseconds)
// ============================================
#define TELEMETRY_INTERVAL 30000   // Send telemetry every 30 seconds
#define LCD_UPDATE_INTERVAL 2000   // Update LCD display every 2 seconds
#define WIFI_RECONNECT_DELAY 5000  // Wait 5 seconds before WiFi reconnect
#define MQTT_RECONNECT_DELAY 5000  // Wait 5 seconds before MQTT reconnect

// ============================================
// Default Thresholds (used if no server config received)
// ============================================
#define DEFAULT_TEMP_ON 28.0
#define DEFAULT_TEMP_OFF 26.0
#define DEFAULT_RH_ON 80.0
#define DEFAULT_RH_OFF 75.0

#endif // CONFIG_H
