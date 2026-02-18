# GPS Lap Timer

**GPS-based lap timer for circuit racing on ESP32-S3**

---

## Overview

A high-performance GPS lap timer designed for circuit racing, featuring:
- Real-time delta time calculation against reference laps
- 3-sector timing with live sector deltas
- 640×172 color display with custom UI
- Simulation mode for development
- SPIFFS-based lap data storage

### Hardware

- **Board:** Waveshare ESP32-S3-Touch-LCD-3.49
- **Display:** 3.49" 640×172 AXS15231B (QSPI)
- **GPS:** u-blox G10A-F33 (10 Hz, UBX protocol)
- **Framework:** ESP-IDF v5.5.2

### Key Features

✅ **Sector Timing** - 3 sectors with individual delta tracking
✅ **Delta Display** - Time delta (center) + speed delta (bar)
✅ **Simulation Mode** - Replay track data for testing
✅ **Reference Lap** - Compare against best lap or custom reference
✅ **Serial Commands** - Full CLI for control and debugging

---

## Quick Start

### 1. Prerequisites

- ESP-IDF v5.5.2 (get from [Espressif](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/))
- USB cable for flashing and power
- Optional: GPS module for real track testing

### 2. Build

```bash
# Set target (first time only)
idf.py set-target esp32s3

# Build
idf.py build

# Flash and monitor
idf.py -p COM3 flash monitor
```

**Full build instructions:** [docs/reference/build-environment.md](docs/reference/build-environment.md)

### 3. First Run

After flashing, the device boots in **GPS hardware mode**. Switch to Simulation mode with the `e` serial command to test without a GPS module (replays Everland track data).

**Serial commands:**
- `e` - Switch between GPS and Simulation modes
- `h` - Show help with all commands
- `s` - Show status

**See:** [docs/guides/quick-start.md](docs/guides/quick-start.md) for quick reference

---

## Documentation

📚 **Complete documentation:** [docs/README.md](docs/README.md)

### Essential Reading

| Document | Description |
|----------|-------------|
| [docs/reference/architecture.md](docs/reference/architecture.md) | System architecture |
| [docs/reference/hardware.md](docs/reference/hardware.md) | Hardware specifications |
| [docs/reference/ui-specification.md](docs/reference/ui-specification.md) | UI layout and design |
| [docs/reference/data-structures.md](docs/reference/data-structures.md) | Data structures reference |
| [docs/planning/roadmap.md](docs/planning/roadmap.md) | Feature roadmap |

---

## Project Structure

```
gps_laptimer/
├── main/                # Main application code
├── components/          # Modular components (geo, track, timing, modes)
├── docs/                # 📚 Documentation (SSOT)
│   ├── reference/       # Architecture, hardware, UI, data structures
│   ├── planning/        # Roadmap, features backlog
│   ├── development/     # Status, changelog
│   └── guides/          # How-to guides
├── examples/            # Reference lap data (Everland track)
└── CMakeLists.txt       # ESP-IDF build config
```

---

## Current Status (v1.1.0, 2026-02-19)

**Implemented:**
- ✅ Sector timing (distance-based, per-track layout)
- ✅ Delta calculation (time + speed, Kalman-smoothed)
- ✅ Page system (10 pages: ModeSelect, WaitGPS, PreTrack, Laptimer, Emulation, GPSStatus, Settings, BleOTA, PhonePlate, Transition)
- ✅ Multi-track support (Everland full/short, Inje Speedium full/south/north)
- ✅ Track auto-detection via finish-line crossing at 60+ km/h
- ✅ Sensor fusion (Wahba SVD axis calibrator + 1D Speed Kalman filter)
- ✅ LVGL UI (640×172 landscape, 60 Hz, gesture support)
- ✅ Simulation mode (Everland track replay)
- ✅ GPS hardware mode (u-blox G10A-F33, 10 Hz, 115200 baud)
- ✅ Per-track best lap storage (SD card with SPIFFS fallback)
- ✅ SD card logging (GPS track CSV + event log)
- ✅ WiFi captive portal (SoftAP "LAPTIMER", settings UI)
- ✅ BLE OTA firmware update
- ✅ Battery monitoring (LiPo discharge curve, SoC%)
- ✅ Auto power-off (5 min backlight off, 30 min power off)

**See:** [docs/planning/roadmap.md](docs/planning/roadmap.md)

---

## Screenshots

### UI Layout (640×172)

```
┌────────────────────────────────────────────────────────────┐
│  S1 -0.42       LAP 01                1:23.4              │
│  S2 +0.72                           BEST 1:27:00          │
│  S3 -0.32        +1.34                                    │
│  ████████████████████████                    +1.2km/h     │
└────────────────────────────────────────────────────────────┘
```

- **Left:** Sector deltas (S1/S2/S3)
- **Center:** Time delta vs reference
- **Right:** Lap number, current time, best time
- **Bottom:** Speed delta bar with text

**Full specification:** [docs/reference/ui-specification.md](docs/reference/ui-specification.md)

---

## Contributing

We follow **SSOT (Single Source of Truth)** for documentation:
- Each fact exists in exactly one place ([docs/reference/](docs/reference/))
- Other docs link to the reference (no duplication)

**Before contributing:**
1. Check [docs/planning/roadmap.md](docs/planning/roadmap.md)
2. Follow coding style (4 spaces, camelCase functions, UPPER_SNAKE_CASE constants)

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Links

- **Hardware Wiki:** https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-3.49
- **ESP-IDF Documentation:** https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/
- **LVGL Documentation:** https://docs.lvgl.io/9.0/
- **u-blox G10 Series:** https://www.u-blox.com/en/product/max-m10-series

---

**📖 For detailed information:** [docs/README.md](docs/README.md)
