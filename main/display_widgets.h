/**
 * @file display_widgets.h
 * @brief Internal accessors for LVGL widgets and display helpers.
 *
 * Widget pointers remain static in waveshare_display.cpp.
 * This header exposes controlled access for page implementations.
 */

#pragma once

#include "lvgl.h"
#include "page.h"

// ── LVGL mutex helpers ──
bool lvglLock(int timeoutMs);
void lvglUnlock();

// ── Gesture polling ──
// Returns the current gesture (consumes the gesture event).
Gesture pollGesture(void);

// ── Cross-page display helpers ──
void updateNotificationDisplay(void);
void updateBatteryWarning(void);

// ── Page visibility ──
// Shows/hides LVGL widgets appropriate for the given page.
void applyPageVisibilityForPage(PageId pageId);

// ── Startup overlay widgets ──
lv_obj_t* getStartupOverlay(void);
lv_obj_t* getStartupTitleLabel(void);
lv_obj_t* getStartupVersionLabel(void);
lv_obj_t* getStartupStatusLabel(void);
lv_obj_t* getStartupHintLabel(void);

// ── Phone plate widgets ──
lv_obj_t* getPhoneOverlay(void);
lv_obj_t* getPhoneNumberLabel(void);
lv_obj_t* getPhoneHintLabel(void);

// ── Main UI widgets ──
lv_obj_t* getDeltaLabel(void);
lv_obj_t* getSpeedDeltaLabel(void);
lv_obj_t* getDatetimeLabel(void);
lv_obj_t* getLapnumLabel(void);
lv_obj_t* getLaptimeLabel(void);
lv_obj_t* getBestLabel(void);
lv_obj_t* getBarBg(void);
lv_obj_t* getBarUp(void);
lv_obj_t* getBarDown(void);
lv_obj_t* getNotificationLabel(void);

// ── Sector delta labels (S1, S2, S3) ──
lv_obj_t* getSectorDeltaLabel(int index);

// ── GPS Status page labels ──
lv_obj_t* getGpsStatusLabel(int line);

// ── Lap complete overlay ──
lv_obj_t* getLapCompleteOverlay(void);
lv_obj_t* getLapCompleteTimeLabel(void);
lv_obj_t* getLapCompleteBorder(void);

// ── Battery warning labels ──
lv_obj_t* getBatLowLabel(void);
lv_obj_t* getBatLowStartupLabel(void);
lv_obj_t* getBatLowPhoneLabel(void);

// ── Laptimer/Emulation shared display functions ──
// (extracted from updateLapData / updateTimeData / updateGpsData)
void updateLaptimerDisplay(int userPageIndex);
void updateLaptimerTimeDisplay(void);
void updateLaptimerGpsDisplay(void);

// ── GPS Status page update ──
void updateGpsStatusDisplay(void);

// ── Pre-track display ──
void updatePreTrackDisplay(const char* trackName);
