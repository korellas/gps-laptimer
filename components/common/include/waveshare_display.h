/**
 * @file waveshare_display.h
 * @brief UI layer header for Waveshare ESP32-S3-Touch-LCD-3.49 (ESP-IDF)
 * @version 5.0 - HAL/UI split (HAL API in display_hal.h)
 */

#ifndef WAVESHARE_DISPLAY_H
#define WAVESHARE_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// Frame data structures (shared with other modules)
extern struct tFrame tframe;
extern struct gFrame gframe;
extern struct lFrame lframe;
extern bool gvalid, tvalid, lvalid;

// Initialization (UI layer — call after initDisplay())
void setupUI(void);

// GPS signal status
void setGpsSignalLost(bool lost);
bool isGpsSignalLost(void);

// Notifications
void showNotification(const char* message, uint16_t durationMs);

// Data updates (internal, called by display helper functions)
void updateLapData(void);
void updateGpsData(void);
void updateTimeData(void);

// Delta history reset
void resetDeltaHistory(void);

// Startup screen (creates LVGL widgets, called once during init)
void createStartupScreen(void);

// Display test
void displayTest(void);

#ifdef __cplusplus
}
#endif

#endif // WAVESHARE_DISPLAY_H
