# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Target Hardware: Waveshare ESP32-S3-Touch-AMOLED-2.06

| Component | Details |
|---|---|
| MCU | ESP32-S3R8, dual-core Xtensa LX7 @ 240MHz |
| RAM | 8MB PSRAM (OPI), 512KB SRAM |
| Flash | 32MB |
| Display | 2.06" AMOLED, 410×502px, driver: **CO5300** (QSPI) |
| Touch | **FT3168** capacitive controller (I2C) |
| IMU | 6-axis accelerometer + gyroscope |
| Audio | ES8311 codec + dual digital mics |
| Power | AXP2101 PMIC, LiPo battery support (MX1.25) |
| RTC | PCF85063 |
| Connectivity | Wi-Fi 802.11 b/g/n, Bluetooth 5 / BLE |
| USB | Type-C |
| Storage | TF card slot |

**Official resources:**
- Wiki: https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-2.06
- GitHub: https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.06
- ESP Component Registry: `waveshare/esp32_s3_touch_amoled_2_06`

## Build Environment

### Arduino (arduino-cli)

arduino-cli is already installed at `/c/Users/mik/AppData/Local/arduino-cli/arduino-cli.exe`.

FQBN: `esp32:esp32:esp32s3`

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32s3 <sketch>/

# Upload (check COM port with: arduino-cli board list)
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32s3 <sketch>/
```

**Required board settings:**
- PSRAM: `OPI PSRAM`
- Flash Mode: `QIO 80MHz`
- Partition Scheme: large app or 16MB Flash layout

### PlatformIO

```ini
[env:waveshare_s3_amoled]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_build.arduino.memory_type = qio_opi
board_upload.flash_size = 32MB
board_build.partitions = default_16MB.csv
build_flags =
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue
```

## Key Libraries

| Library | Purpose | Notes |
|---|---|---|
| LVGL | UI framework | v8 and v9 are NOT cross-compatible; pick one and stick to it |
| Arduino_DriveBus | Display/touch bus abstraction | Required by Waveshare display driver |
| GFX_Library_for_Arduino | Graphics primitives | |
| Waveshare CO5300 driver | CO5300 QSPI display driver | From official sample package |

**LVGL setup:**
- Copy `lv_conf_template.h` → `lv_conf.h` in Arduino libraries root (same level as `lvgl/` folder)
- Change first `#if 0` to `#if 1`
- Use the pre-configured `lv_conf.h` from Waveshare sample package when available

Custom Waveshare libraries are NOT in Arduino Library Manager — install from the official GitHub sample package manually.

## Hardware Notes

- 5V via Type-C required to power the display (USB power bank or PC USB-A may be insufficient)
- AXP2101 PMIC controls power rails — display will not initialize without proper PMIC setup
- Install a LiPo battery to keep the PCF85063 RTC running without USB power
- Display uses QSPI (not SPI or MIPI) — do not use generic ST7789/RM67162 driver code

## Related Project

AMY_ESP32 (`../AMY_ESP32`) — ESP32-S2 TFT weather display using same arduino-cli toolchain.
