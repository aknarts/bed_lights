# Zigbee Multi-Channel Color Dimmable Light (ESP32‑C6)

Firmware for an ESP32‑C6 acting as a Zigbee Router exposing multiple independent Color Dimmable Light endpoints (one per physical LED channel). Supports On/Off, Level, Color (XY, Hue/Sat, Color Temperature) and Identify effects per channel (blink, breathe, ICU, random color).

## Current Channel Layout
- 12 stair LEDs (single pixel each) → endpoints 1–12
- 2 bed strips (multi‑pixel each, default 60 LEDs) → endpoints 13–14
(Base endpoint = 1, total = 14)

Adjust counts/pins in: bed_lights.h (STAIRS_LED_COUNT, BED_STRIP_COUNT, BED_STRIP_LED_LENGTH) and the channel_cfg array in app_main() inside bed_lights.c.

## Features
- Multiple HA Color Dimmable Light endpoints (ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID)
- Clusters per endpoint: Basic, Identify (srv+cli), On/Off, Level, Color Control, Scenes, Groups
- Color modes: XY, Hue/Sat, Color Temperature (153–500 mired clamp) + enhanced hue placeholder
- Per‑channel effect engine (FreeRTOS task per active effect)
- Reporting: On/Off + Level per endpoint

## Files
- main/bed_lights.c – Multi-endpoint Zigbee setup, attribute dispatch → channel driver
- main/bed_lights.h – Configuration constants (channel counts, base endpoint)
- main/light_driver.c/.h – Multi-channel LED driver + effects

Legacy (not compiled, safe to delete): ultrasonic.*, temp_sensor_driver.*, ws2812fx_stub.*

## Build
```bash
idf.py set-target esp32c6
idf.py build
idf.py flash monitor
```

## Customization
1. Change channel GPIO & length in channel_cfg (app_main).
2. Add/remove channels: update STAIRS_LED_COUNT / BED_STRIP_COUNT and channel_cfg; TOTAL_LIGHT_CHANNELS auto-adjusts.
3. Effects: extend light_effect_t + effect_task_ch logic.
4. Performance: large strips may need higher task stack or DMA alternative (e.g. RMT limitations).

## Notes / Limits
- RMT channel availability may cap simultaneous strips; using many long strips can increase refresh latency.
- No state persistence yet (consider NVS for power/level/color per channel).
- Scenes cluster present but not yet storing per-channel custom scenes in NVS.
- Basic cluster duplicated per endpoint (could be optimized to a single endpoint or manufacturer/model omitted on secondary endpoints if spec allows—left for clarity).

## Next Steps (Optional)
- Persist per-channel state
- Smooth transitions / fade engine
- Group effects spanning multiple channels (e.g. stair chase)
- Dynamic reconfiguration over a custom cluster or OTA update

## Licensing
Espressif example base: CC0-1.0. Additions keep same.
