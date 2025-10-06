/**
 * FarmOS ESP32 Firmware - Smart Greenhouse Controller
 * 
 * Features:
 * - Temperature & Humidity monitoring (DHT22 or SHT31)
 * - Fan control with hysteresis
 * - MQTT connectivity over Internet
 * - Threshold configuration via MQTT
 * - NVS storage for persistence
 * - Manual fan override
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>

#if defined(SENSOR_DHT22) || defined(SENSOR_DHT11)
  #include <DHT.h>
#endif

#ifdef SENSOR_SHT31
  #include <Adafruit_SHT31.h>
#endif

#ifdef USE_LCD_I2C
  #include <LiquidCrystal_I2C.h>
#endif

#include "config.h"

// Include TLS support if MQTTS is enabled
#if defined(MQTT_USE_TLS) && MQTT_USE_TLS
  #include <WiFiClientSecure.h>
  #include "certificates.h"
  WiFiClientSecure espClient;
#else
  WiFiClient espClient;
#endif

PubSubClient mqtt(espClient);
Preferences preferences;

#ifdef SENSOR_DHT22
  DHT dht(DHT_PIN, DHT22);
#endif

#ifdef SENSOR_DHT11
  DHT dht(DHT_PIN, DHT11);
#endif

#ifdef SENSOR_SHT31
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
#endif

#ifdef USE_LCD_I2C
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
#endif

// State variables
struct Thresholds {
  float temp_on;
  float temp_off;
  float rh_on;
  float rh_off;
} thresholds;

enum FanMode {
  AUTO,
  MANUAL_ON,
  MANUAL_OFF
};

FanMode fanMode = AUTO;
bool fanState = false;
unsigned long lastTelemetry = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long bootTime = 0;
unsigned long lastLcdUpdate = 0;

// MQTT Topics
String topicTelemetry;
String topicState;
String topicAck;
String topicCmdThresholds;
String topicCmdFan;

// Function declarations
void setupWiFi();
void setupMQTT();
void reconnectWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishTelemetry();
void publishState();
void publishAck(const char* cmd, bool ok);
void readSensors(float &temp, float &humidity);
void controlFan(float temp, float humidity);
void loadThresholds();
void saveThresholds();
void handleThresholdCommand(JsonDocument &doc);
void handleFanCommand(JsonDocument &doc);
void updateLCD(float temp, float humidity);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n╔══════════════════════════════════════════════════════════╗");
  Serial.println("║         TerrAI ESP32 Firmware v1.1.0                    ║");
  Serial.println("║         Smart growth. Intelligent earth.                ║");
  Serial.println("║         by OMARINO IT Services                           ║");
  Serial.println("╚══════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  #if defined(MQTT_USE_TLS) && MQTT_USE_TLS
    Serial.println("Mode: MQTTS (Encrypted)");
  #else
    Serial.println("Mode: MQTT (Local)");
  #endif
  Serial.println();
  
  bootTime = millis();
  
  // Initialize GPIO
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize LCD
  #ifdef USE_LCD_I2C
    Serial.println("Initializing LCD display...");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TerrAI ESP32");
    lcd.setCursor(0, 1);
    lcd.print("Autonomous Mode");
    delay(2000);
  #endif
  
  // Initialize sensor
  #ifdef SENSOR_DHT22
    Serial.println("Initializing DHT22 sensor...");
    dht.begin();
  #endif
  
  #ifdef SENSOR_DHT11
    Serial.println("Initializing DHT11 sensor...");
    dht.begin();
  #endif
  
  #ifdef SENSOR_SHT31
    Serial.println("Initializing SHT31 sensor...");
    if (!sht31.begin(0x44)) {
      Serial.println("ERROR: Could not find SHT31 sensor!");
      // Continue anyway, will retry in loop
    }
  #endif
  
  // Load thresholds from NVS
  loadThresholds();
  
  // Setup MQTT topics
  topicTelemetry = String("terrai/") + DEVICE_ID + "/telemetry";
  topicState = String("terrai/") + DEVICE_ID + "/state";
  topicAck = String("terrai/") + DEVICE_ID + "/ack";
  topicCmdThresholds = String("terrai/") + DEVICE_ID + "/cmd/thresholds";
  topicCmdFan = String("terrai/") + DEVICE_ID + "/cmd/fan";
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup MQTT
  setupMQTT();
  
  Serial.println("\n=================================");
  Serial.println("Setup complete!");
  Serial.println("=================================");
  Serial.println("Mode: AUTONOMOUS (edge intelligence)");
  Serial.println("- Sensors: Reading locally");
  Serial.println("- Fan control: Using saved thresholds");
  Serial.println("- Network: Will retry in background");
  Serial.println("=================================\n");
  Serial.println("Starting main loop...\n");
}

void loop() {
  // Try to maintain WiFi connection (non-blocking)
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWiFiAttempt > WIFI_RECONNECT_DELAY) {
      reconnectWiFi();
      lastWiFiAttempt = millis();
    }
    // Don't return - continue with autonomous operation
  }
  
  // Try to maintain MQTT connection (non-blocking)
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    if (millis() - lastMqttAttempt > MQTT_RECONNECT_DELAY) {
      reconnectMQTT();
      lastMqttAttempt = millis();
    }
    // Don't return - continue with autonomous operation
  }
  
  // Process MQTT messages if connected
  if (mqtt.connected()) {
    mqtt.loop();
  }
  
  // Read sensors and control fan (ALWAYS, even without WiFi/MQTT)
  float temp, humidity;
  readSensors(temp, humidity);
  
  if (!isnan(temp) && !isnan(humidity)) {
    // AUTONOMOUS CONTROL: Works offline with saved thresholds
    controlFan(temp, humidity);
    
    // Update LCD display periodically
    #ifdef USE_LCD_I2C
      if (millis() - lastLcdUpdate > LCD_UPDATE_INTERVAL) {
        updateLCD(temp, humidity);
        lastLcdUpdate = millis();
      }
    #endif
  }
  
  // Publish telemetry periodically (only if connected)
  if (mqtt.connected() && millis() - lastTelemetry > TELEMETRY_INTERVAL) {
    if (!isnan(temp) && !isnan(humidity)) {
      publishTelemetry();
    }
    lastTelemetry = millis();
  }
  
  delay(100);
}

void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Non-blocking: Try for 5 seconds, then continue
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("\nWiFi connection failed - running in AUTONOMOUS mode");
    Serial.println("Will retry connection in background...");
  }
}

void reconnectWiFi() {
  Serial.println("Reconnecting to WiFi...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void setupMQTT() {
  // Configure TLS if enabled
  #if defined(MQTT_USE_TLS) && MQTT_USE_TLS
    Serial.println("🔐 Configuring MQTTS (TLS encryption)...");
    #ifdef SKIP_CERT_VERIFICATION
      Serial.println("⚠️  WARNING: Certificate verification DISABLED!");
      Serial.println("   This is INSECURE - only for testing!");
      espClient.setInsecure();
    #else
      Serial.println("   Using Let's Encrypt root CA");
      espClient.setCACert(root_ca);
    #endif
  #else
    Serial.println("📡 Using standard MQTT (no encryption)");
  #endif

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(15);
  
  Serial.print("MQTT Broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  
  // Non-blocking: Only try if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    reconnectMQTT();
  } else {
    Serial.println("Skipping MQTT setup - no WiFi connection");
    Serial.println("MQTT will connect automatically once WiFi is available");
  }
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  #if defined(MQTT_USE_TLS) && MQTT_USE_TLS
    Serial.print("🔌 Connecting to MQTTS broker: ");
  #else
    Serial.print("🔌 Connecting to MQTT broker: ");
  #endif
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  
  String clientId = String("terrai-") + DEVICE_ID;
  
  // Connect with or without authentication
  bool connected = false;
  #if defined(MQTT_USER) && defined(MQTT_PASS)
    connected = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
  #else
    connected = mqtt.connect(clientId.c_str());
  #endif
  
  if (connected) {
    Serial.println("✅ MQTT connected!");
    
    // Subscribe to command topics
    mqtt.subscribe(topicCmdThresholds.c_str());
    mqtt.subscribe(topicCmdFan.c_str());
    
    Serial.println("   Subscribed to command topics");
    
    // Publish state message
    publishState();
    
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(mqtt.state());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  
  // Parse JSON payload
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  String topicStr = String(topic);
  
  if (topicStr == topicCmdThresholds) {
    handleThresholdCommand(doc);
  } else if (topicStr == topicCmdFan) {
    handleFanCommand(doc);
  }
}

void handleThresholdCommand(JsonDocument &doc) {
  Serial.println("Received threshold update");
  
  thresholds.temp_on = doc["temp_on"] | DEFAULT_TEMP_ON;
  thresholds.temp_off = doc["temp_off"] | DEFAULT_TEMP_OFF;
  thresholds.rh_on = doc["rh_on"] | DEFAULT_RH_ON;
  thresholds.rh_off = doc["rh_off"] | DEFAULT_RH_OFF;
  
  Serial.printf("New thresholds: temp_on=%.1f, temp_off=%.1f, rh_on=%.1f, rh_off=%.1f\n",
    thresholds.temp_on, thresholds.temp_off, thresholds.rh_on, thresholds.rh_off);
  
  saveThresholds();
  publishAck("thresholds", true);
}

void handleFanCommand(JsonDocument &doc) {
  const char* mode = doc["mode"];
  
  Serial.print("Received fan command: mode=");
  Serial.println(mode);
  
  if (strcmp(mode, "auto") == 0) {
    fanMode = AUTO;
    Serial.println("Fan set to AUTO mode");
  } else if (strcmp(mode, "manual") == 0) {
    bool state = doc["state"] | false;
    fanMode = state ? MANUAL_ON : MANUAL_OFF;
    fanState = state;
    digitalWrite(FAN_PIN, state ? HIGH : LOW);
    Serial.printf("Fan set to MANUAL mode: %s\n", state ? "ON" : "OFF");
  }
  
  publishAck("fan", true);
}

void publishTelemetry() {
  float temp, humidity;
  readSensors(temp, humidity);
  
  if (isnan(temp) || isnan(humidity)) {
    Serial.println("ERROR: Invalid sensor readings");
    return;
  }
  
  StaticJsonDocument<256> doc;
  doc["ts"] = millis() + bootTime;
  doc["temperature_c"] = round(temp * 10) / 10.0;
  doc["humidity_pct"] = round(humidity * 10) / 10.0;
  doc["fan"] = fanState;
  doc["rssi"] = WiFi.RSSI();
  doc["fw"] = FIRMWARE_VERSION;
  
  String payload;
  serializeJson(doc, payload);
  
  if (mqtt.publish(topicTelemetry.c_str(), payload.c_str())) {
    Serial.println("Telemetry published:");
    Serial.println(payload);
  } else {
    Serial.println("ERROR: Failed to publish telemetry");
  }
}

void publishState() {
  StaticJsonDocument<128> doc;
  doc["ip"] = WiFi.localIP().toString();
  doc["uptime_s"] = millis() / 1000;
  doc["heap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  mqtt.publish(topicState.c_str(), payload.c_str());
  Serial.println("State published");
}

void publishAck(const char* cmd, bool ok) {
  StaticJsonDocument<64> doc;
  doc["cmd"] = cmd;
  doc["ok"] = ok;
  doc["applied"] = true;
  
  String payload;
  serializeJson(doc, payload);
  
  mqtt.publish(topicAck.c_str(), payload.c_str());
  Serial.printf("Ack published: cmd=%s, ok=%d\n", cmd, ok);
}

void readSensors(float &temp, float &humidity) {
  #if defined(SENSOR_DHT22) || defined(SENSOR_DHT11)
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
  #endif
  
  #ifdef SENSOR_SHT31
    temp = sht31.readTemperature();
    humidity = sht31.readHumidity();
  #endif
}

void controlFan(float temp, float humidity) {
  // Skip if in manual mode
  if (fanMode != AUTO) {
    return;
  }
  
  bool shouldBeOn = fanState;
  
  // Hysteresis logic:
  // Turn ON if temp > temp_on OR humidity > rh_on
  // Turn OFF only when temp < temp_off AND humidity < rh_off
  
  if (!fanState) {
    // Fan is OFF, check if we should turn it ON
    if (temp > thresholds.temp_on || humidity > thresholds.rh_on) {
      shouldBeOn = true;
      Serial.printf("Fan ON: T=%.1f°C (threshold=%.1f), H=%.1f%% (threshold=%.1f)\n",
        temp, thresholds.temp_on, humidity, thresholds.rh_on);
    }
  } else {
    // Fan is ON, check if we should turn it OFF
    if (temp < thresholds.temp_off && humidity < thresholds.rh_off) {
      shouldBeOn = false;
      Serial.printf("Fan OFF: T=%.1f°C (threshold=%.1f), H=%.1f%% (threshold=%.1f)\n",
        temp, thresholds.temp_off, humidity, thresholds.rh_off);
    }
  }
  
  if (shouldBeOn != fanState) {
    fanState = shouldBeOn;
    digitalWrite(FAN_PIN, fanState ? HIGH : LOW);
  }
}

void loadThresholds() {
  preferences.begin("terrai", false);
  
  thresholds.temp_on = preferences.getFloat("temp_on", DEFAULT_TEMP_ON);
  thresholds.temp_off = preferences.getFloat("temp_off", DEFAULT_TEMP_OFF);
  thresholds.rh_on = preferences.getFloat("rh_on", DEFAULT_RH_ON);
  thresholds.rh_off = preferences.getFloat("rh_off", DEFAULT_RH_OFF);
  
  preferences.end();
  
  Serial.println("Loaded thresholds from NVS:");
  Serial.printf("  temp_on=%.1f, temp_off=%.1f, rh_on=%.1f, rh_off=%.1f\n",
    thresholds.temp_on, thresholds.temp_off, thresholds.rh_on, thresholds.rh_off);
}

void saveThresholds() {
  preferences.begin("terrai", false);
  
  preferences.putFloat("temp_on", thresholds.temp_on);
  preferences.putFloat("temp_off", thresholds.temp_off);
  preferences.putFloat("rh_on", thresholds.rh_on);
  preferences.putFloat("rh_off", thresholds.rh_off);
  
  preferences.end();
  
  Serial.println("Thresholds saved to NVS");
}

void updateLCD(float temp, float humidity) {
  #ifdef USE_LCD_I2C
    lcd.clear();
    
    // Line 1: Temperature and Humidity
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temp, 1);
    lcd.print("C H:");
    lcd.print(humidity, 0);
    lcd.print("%");
    
    // Line 2: Fan status and connection status
    lcd.setCursor(0, 1);
    lcd.print("Fan:");
    lcd.print(fanState ? "ON " : "OFF");
    
    // Show connection status (or OFFLINE mode)
    lcd.setCursor(8, 1);
    if (WiFi.status() == WL_CONNECTED) {
      if (mqtt.connected()) {
        lcd.print("ONLINE");
      } else {
        lcd.print("WiFi:OK");
      }
    } else {
      lcd.print("OFFLINE");
    }
  #endif
}
