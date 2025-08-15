# Depth Sensor (ESP32‑C6 + Zigbee)

Water tank level sensor using an ESP32‑C6 and a JSN‑SR04T v3 waterproof ultrasonic module.  
Publishes distance (cm) and board temperature via Zigbee (Home Assistant through Zigbee2MQTT).  
An onboard LED is exposed via Zigbee for basic status/identify.

## Features
- Ultrasonic distance reporting (Analog Output cluster presentValue in cm).
- Temperature reporting (Temperature Measurement cluster, 0.01°C resolution).
- Identify effect blinks the onboard LED for visual feedback.
- Robust ultrasonic handling with retries and timing fixes for JSN‑SR04T v3.

## Hardware
- MCU: ESP32‑C6
- Ultrasonic sensor: JSN‑SR04T v3
  - Power: directly from the ESP32‑C6 board headers (no external supply needed on this board)
  - Logic level: this project assumes your board/sensor combo is 3.3V‑compatible or already level‑shifted
    - If you use a different JSN‑SR04T variant with a strict 5V echo, add level shifting to protect the GPIO
  - Common GND between sensor and ESP32‑C6
  - Mount away from obstructions; avoid very near (< ~25 cm) targets and angled/absorbent surfaces
  - Simple two‑board setup: ESP32‑C6 dev board + JSN‑SR04T v3
- Onboard LED: single LED (mono) or discrete RGB depending on board version (driver supports both)
- Temperature sensor: on-board sensor (software driver included)

## ESP‑IDF Setup

Prerequisites:
- ESP‑IDF installed (IDF 5.x recommended) and required tools.

Environment:
