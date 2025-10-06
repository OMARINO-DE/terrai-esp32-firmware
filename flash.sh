#!/bin/bash
# FarmOS ESP32 Quick Reflash Script
# Use this after updating WiFi credentials in config.h

ESP32_PORT="/dev/cu.usbserial-0001"
FIRMWARE_DIR="apps/firmware/esp32"

echo "🌱 FarmOS ESP32 Flash Tool"
echo "=========================="
echo ""

# Check if we're in the right directory
if [ ! -f "platformio.ini" ]; then
  cd "$FIRMWARE_DIR" 2>/dev/null || {
    echo "❌ Error: Run this from FarmOS root or firmware directory"
    exit 1
  }
fi

# Check if config.h exists
if [ ! -f "src/config.h" ]; then
  echo "❌ Error: config.h not found!"
  echo "   Create it from: src/config.h.example"
  exit 1
fi

# Check for placeholder WiFi credentials
if grep -q "YOUR_WIFI_SSID" src/config.h; then
  echo "⚠️  WARNING: WiFi credentials not configured!"
  echo ""
  echo "Please edit: src/config.h"
  echo "Update lines 11-12 with your WiFi name and password"
  echo ""
  read -p "Continue anyway? (y/N) " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
  fi
fi

# Check if ESP32 is connected
if [ ! -e "$ESP32_PORT" ]; then
  echo "❌ Error: ESP32 not found at $ESP32_PORT"
  echo "   Available ports:"
  ls -1 /dev/cu.* 2>/dev/null | grep -E "(usb|SLAB|wchusbserial)" || echo "   None found"
  exit 1
fi

echo "✅ Configuration:"
echo "   ESP32 Port: $ESP32_PORT"
echo "   Server IP: 192.168.60.166"
echo "   Device ID: gh-esp32"
echo ""
echo "📡 Starting flash process..."
echo ""

# Flash the firmware
platformio run --target upload --upload-port "$ESP32_PORT"

if [ $? -eq 0 ]; then
  echo ""
  echo "✅ Flash successful!"
  echo ""
  echo "🖥️  To monitor serial output:"
  echo "   platformio device monitor --port $ESP32_PORT --baud 115200"
  echo ""
  echo "🌐 Dashboard: http://192.168.60.166:8080"
  echo "   Look for device: gh-esp32"
  echo ""
else
  echo ""
  echo "❌ Flash failed! Check:"
  echo "   - ESP32 is connected"
  echo "   - No other program is using serial port"
  echo "   - USB cable supports data transfer"
  exit 1
fi
