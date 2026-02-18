/**
 * @file waveshare_display.h
 * @brief Display driver header for Waveshare ESP32-S3-Touch-LCD-3.49 (ESP-IDF)
 * @version 3.0 - ESP-IDF + LVGL
 */

#ifndef WAVESHARE_DISPLAY_H
#define WAVESHARE_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.hpp"

// Forward declaration (avoid pulling driver/i2c_master.h into all consumers)
typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

// Frame data structures (shared with other modules)
extern struct tFrame tframe;
extern struct gFrame gframe;
extern struct lFrame lframe;
extern bool gvalid, tvalid, lvalid;

// Early power latch (call before any other init)
void initPowerLatch(void);

// Initialization
void initDisplay(void);
void setupUI(void);

// GPS signal status
void setGpsSignalLost(bool lost);
bool isGpsSignalLost(void);

// Notifications
void showNotification(const char* message, uint16_t durationMs);

// Main loop
void displayLoop(void);

// Data updates (called when data changes)
void updateLapData(void);
void updateGpsData(void);
void updateTimeData(void);

// Touch handling
bool readTouch(void);

// Delta history reset
void resetDeltaHistory(void);

// PRE_TRACK mode (속도+시각 표시, 세션 시작 전)
void setPreTrackMode(bool enabled, const char* trackName = nullptr);
bool isPreTrackMode(void);

// Startup screen
void createStartupScreen(void);
void updateStartupScreen(void);
void dismissStartupScreen(void);
bool isStartupScreenActive(void);

// Sensor I2C bus (GPIO47/48, shared with TCA9554/RTC/IMU)
i2c_master_bus_handle_t getSensorI2CBus(void);

// Power management
void systemPowerOff(void);

// GPS Status only mode (시작 화면에서 GPS STATUS 선택 시 — 랩타이머 비활성)
bool isGpsStatusOnlyMode(void);

// Display test
void displayTest(void);


#ifdef __cplusplus
}
#endif

#endif // WAVESHARE_DISPLAY_H
