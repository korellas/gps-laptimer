/**
 * @file display_hal.h
 * @brief Hardware Abstraction Layer for Waveshare ESP32-S3-Touch-LCD-3.49
 *
 * LCD panel, touch, backlight, LVGL infrastructure, I2C bus, power control.
 * main/ only header — components/ cannot include this (section 4.4).
 * Page implementations access HAL API via display_widgets.h (transitive).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Forward declaration
typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

// ── Display resolution (after rotation to landscape) ──
#define LCD_H_RES  640
#define LCD_V_RES  172

// ── Initialization (call in order) ──
void initPowerLatch(void);
void initDisplay(void);

// ── LVGL mutex (for UI thread safety) ──
bool lvglLock(int timeoutMs);  // timeoutMs < 0 => portMAX_DELAY
void lvglUnlock(void);

// ── Backlight ──
void setBacklight(bool on);
bool isBacklightOn(void);

// ── Touch ──
bool readTouch(void);
bool readTouchXY(int16_t* x, int16_t* y);

// ── I2C sensor bus (shared GPIO47/48: TCA9554, RTC, IMU) ──
i2c_master_bus_handle_t getSensorI2CBus(void);

// ── Power ──
void systemPowerOff(void);

#ifdef __cplusplus
}
#endif
