/**
 * @file waveshare_display.cpp
 * @brief UI layer for Waveshare ESP32-S3-Touch-LCD-3.49 (ESP-IDF + LVGL)
 * @version 4.0 - HAL/UI split
 *
 * UI widget creation/update, gesture detection, notifications, page rendering.
 * Hardware abstraction is in display_hal.cpp.
 */

#include "waveshare_display.h"
#include "display_hal.h"
#include "display_widgets.h"
#include "config.h"
#include "types.h"
#include "protocol.hpp"
#include "sector_timing.h"
#include "ublox_gps.h"
#include "display_config.h"
#include "sensor_fusion.h"

#include <cstring>
#include <cstdio>
#include <cmath>
#include <ctime>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"

// Share Tech Mono font declarations
LV_FONT_DECLARE(share_tech_mono_24);
LV_FONT_DECLARE(share_tech_mono_32);
LV_FONT_DECLARE(share_tech_mono_56);
LV_FONT_DECLARE(share_tech_mono_72);

static const char *TAG = "DISPLAY_UI";

// LVGL UI elements
static lv_obj_t *lbl_laptime = NULL;
static lv_obj_t *lbl_lapnum = NULL;
static lv_obj_t *lbl_best = NULL;
static lv_obj_t *lbl_delta = NULL;
static lv_obj_t *lbl_center_unit = NULL;  // "km/h" unit label (speed mode only)
static lv_obj_t *lbl_speed_delta = NULL;
static lv_obj_t *bar_bg = NULL;
static lv_obj_t *bar_up = NULL;
static lv_obj_t *bar_down = NULL;
static lv_obj_t *lbl_notification = NULL;

// 저전력 경고 텍스트
static lv_obj_t *lbl_bat_low = NULL;          // main screen
static lv_obj_t *lbl_bat_low_startup = NULL;  // startup overlay
static lv_obj_t *lbl_bat_low_phone = NULL;    // phone overlay

// 배터리 아이콘 (startup overlay)
static lv_obj_t *bat_icon_body = NULL;
static lv_obj_t *bat_icon_tip  = NULL;
static lv_obj_t *bat_icon_fill = NULL;

// 날짜/시간 라벨 (delta 위, 중앙)
static lv_obj_t *lbl_datetime = NULL;

// Sector delta labels (displayed under lap time)
static lv_obj_t *lbl_sector_deltas[3] = {NULL, NULL, NULL};  // S1, S2, S3

// GPS Status page labels
static constexpr int GPS_STATUS_LINE_COUNT = 5;
static constexpr int GPS_STATUS_LINE_WIDTH = 96;
static lv_obj_t *gps_status_labels[GPS_STATUS_LINE_COUNT] = {};

// Lap completion fullscreen overlay
static lv_obj_t *lap_complete_overlay = NULL;
static lv_obj_t *lap_complete_time_label = NULL;
static lv_obj_t *lap_complete_border = NULL;

// ── Lap Summary page overlay ──
#define SUMMARY_ROWS        7   // 1 header + 6 data rows
#define SUMMARY_SEC_COLS    6   // sector columns visible at once
#define SUMMARY_TOTAL_COLS 10   // LAP + TIME + S0..S5 + MIN + MAX

static lv_obj_t* s_summaryOverlay = NULL;
static lv_obj_t* s_summaryCells[SUMMARY_ROWS][SUMMARY_TOTAL_COLS] = {};

// ── IMU Status page overlay ──
static lv_obj_t* s_imuOverlay = NULL;
static lv_obj_t* s_gfDot = NULL;          // G-force moving dot
static constexpr int GF_SIZE = 160;       // G-force circle area (px)
static constexpr float GF_SCALE = 65.0f;  // pixels per 1G
static constexpr int IMU_INFO_LINES = 5;
static lv_obj_t* s_imuLabels[IMU_INFO_LINES] = {};
static lv_obj_t* s_zBar = NULL;            // Z-axis vertical bar container
static lv_obj_t* s_zDot = NULL;            // Z-axis moving dot
static constexpr int ZB_HEIGHT = 160;      // Z bar height (matches G-force)
static constexpr int ZB_WIDTH  = 16;       // Z bar width

static const int SUMMARY_COL_X[SUMMARY_TOTAL_COLS] = {
    4, 46, 132, 194, 256, 318, 380, 442, 506, 566
};
static const int SUMMARY_ROW_Y[SUMMARY_ROWS] = {
    2, 28, 52, 76, 100, 124, 148
};

// Startup screen overlay (화면: STARTUP)
static lv_obj_t *startup_overlay = NULL;
static lv_obj_t *startup_title_label = NULL;
static lv_obj_t *startup_version_label = NULL;
static lv_obj_t *startup_status_label = NULL;
static lv_obj_t *startup_hint_label = NULL;

// Phone plate overlay (화면: PHONE_PLATE)
static lv_obj_t *phone_overlay = NULL;
static lv_obj_t *phone_number_label = NULL;
static lv_obj_t *phone_hint_label = NULL;

// PRE_TRACK 모드 상태 (display helper 함수에서 사용)
static bool s_preTrackMode = false;
static char s_preTrackName[64] = "";
static int s_userPageIndex = 0;  // index into getDisplayConfig().pages[]

// Frame data
struct tFrame tframe = {};
struct gFrame gframe = {};
struct lFrame lframe = {};
bool gvalid = false;
bool tvalid = false;
bool lvalid = false;

// ============================================================
// CACHE FOR SMART UPDATES
// ============================================================

static struct {
    char lapTime[16] = "";
    char lapNum[16] = "";
    char best[64] = "";  // Increased for multi-line top 3 laps
    char delta[16] = "";
    char speedDelta[24] = "";
    char time[8] = "";
    int32_t lastBarValue = -9999;
    bool lastBarPositive = false;
    int8_t lastLowBat = -1;    // 저전력 글로우 상태 (-1=미설정, 0=off, 1=on)
    int8_t lastBatIconPct = -1; // 배터리 아이콘 캐시 (-1=미설정)
} cache;

// ============================================================
// DELTA SMOOTHING
// ============================================================

static float smoothedDelta = 0.0f;
static float smoothedSpeedDeltaKmh = 0.0f;

static const float DELTA_EMA_ALPHA = 0.15f;
static const float SPEED_DELTA_EMA_ALPHA = 0.20f;
static const float SPEED_BAR_RANGE_KMH = 10.0f;
static const float SPEED_BAR_DEADZONE_KMH = 0.25f;


// ============================================================
// UI CREATION
// ============================================================

void setupUI(void)
{
    if (!lvglLock(-1)) return;

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // Delta bar background - covers entire screen (640x172)
    // Green bar: grows from left to right (faster)
    // Red bar: grows from right to left (slower)
    bar_bg = lv_obj_create(scr);
    lv_obj_set_size(bar_bg, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(bar_bg, 0, 0);
    lv_obj_set_style_bg_color(bar_bg, lv_color_black(), 0);
    lv_obj_set_style_border_width(bar_bg, 0, 0);
    lv_obj_set_style_pad_all(bar_bg, 0, 0);
    lv_obj_set_style_radius(bar_bg, 0, 0);

    // ── 저전력 경고 텍스트 (하단 중앙) ──
    lbl_bat_low = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_bat_low, lv_color_hex(0x551111), 0);
    lv_obj_set_style_text_font(lbl_bat_low, &lv_font_montserrat_14, 0);
    lv_label_set_text(lbl_bat_low, "BAT LOW");
    lv_obj_align(lbl_bat_low, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(lbl_bat_low, LV_OBJ_FLAG_HIDDEN);

    // Green bar (faster) - starts from left, grows right
    bar_up = lv_obj_create(bar_bg);
    lv_obj_set_style_bg_color(bar_up, lv_color_make(20, 170, 20), 0);
    // Bright border at the moving edge
    lv_obj_set_style_border_color(bar_up, lv_color_make(40, 220, 40), 0);
    lv_obj_set_style_border_width(bar_up, 2, 0);
    lv_obj_set_style_border_side(bar_up, LV_BORDER_SIDE_RIGHT, 0);
    lv_obj_set_style_radius(bar_up, 0, 0);
    lv_obj_set_size(bar_up, 0, LCD_V_RES);
    lv_obj_set_pos(bar_up, 0, 0);

    // Red bar (slower) - starts from right, grows left
    bar_down = lv_obj_create(bar_bg);
    lv_obj_set_style_bg_color(bar_down, lv_color_make(170, 20, 20), 0);
    // Bright border at the moving edge
    lv_obj_set_style_border_color(bar_down, lv_color_make(220, 40, 40), 0);
    lv_obj_set_style_border_width(bar_down, 2, 0);
    lv_obj_set_style_border_side(bar_down, LV_BORDER_SIDE_LEFT, 0);
    lv_obj_set_style_radius(bar_down, 0, 0);
    lv_obj_set_size(bar_down, 0, LCD_V_RES);
    lv_obj_set_pos(bar_down, LCD_H_RES, 0);

    // 날짜/시간 (delta 위, 중앙, 24pt 흰색)
    lbl_datetime = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_datetime, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_datetime, &share_tech_mono_24, 0);
    lv_obj_set_width(lbl_datetime, 300);
    lv_obj_set_style_text_align(lbl_datetime, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_datetime, LV_ALIGN_TOP_MID, 0, 8);
    lv_label_set_text(lbl_datetime, "");
    lv_obj_add_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);

    // Delta text - largest, centered (overlay on bar)
    lbl_delta = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_delta, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_delta, &share_tech_mono_72, 0);
    lv_obj_set_width(lbl_delta, 420);
    lv_obj_set_style_text_align(lbl_delta, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_delta, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(lbl_delta, "0:00.0");

    // Speed delta text attached to moving bar end
    lbl_speed_delta = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_speed_delta, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_speed_delta, &share_tech_mono_24, 0);
    lv_label_set_text(lbl_speed_delta, "+0.0km/h");
    lv_obj_add_flag(lbl_speed_delta, LV_OBJ_FLAG_HIDDEN);

    // km/h unit label: shown alongside large speed number (speed mode only)
    // x=+120: right of speed digits (3-digit right edge ~+56px from center)
    // y=+8: bottom-aligns 56px font with 72px speed font ((72-56)/2 = 8)
    lbl_center_unit = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_center_unit, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_center_unit, &share_tech_mono_56, 0);
    lv_obj_set_width(lbl_center_unit, 130);
    lv_obj_set_style_text_align(lbl_center_unit, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(lbl_center_unit, LV_ALIGN_CENTER, 120, 8);
    lv_label_set_text(lbl_center_unit, "km/h");
    lv_obj_add_flag(lbl_center_unit, LV_OBJ_FLAG_HIDDEN);

    // ============================================================
    // MAIN UI ELEMENTS
    // ============================================================

    // Lap tag (top-left): "LAP 01"
    lbl_lapnum = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_lapnum, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_lapnum, &share_tech_mono_32, 0);
    lv_obj_align(lbl_lapnum, LV_ALIGN_TOP_LEFT, 8, 4);
    lv_label_set_text(lbl_lapnum, "LAP 01");

    // Lap time label (top-right)
    lbl_laptime = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_laptime, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_laptime, &share_tech_mono_32, 0);
    lv_obj_set_width(lbl_laptime, 230);
    lv_obj_set_style_text_align(lbl_laptime, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(lbl_laptime, LV_ALIGN_TOP_RIGHT, -8, 4);
    lv_label_set_text(lbl_laptime, "0:00.0");
    lv_obj_add_flag(lbl_laptime, LV_OBJ_FLAG_HIDDEN);

    // Best lap label (fixed row under lap row)
    lbl_best = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_best, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_best, &share_tech_mono_24, 0);
    lv_obj_set_width(lbl_best, 320);
    lv_obj_set_style_text_align(lbl_best, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(lbl_best, LV_ALIGN_TOP_RIGHT, -8, 44);
    lv_label_set_text(lbl_best, "BEST --:--:--");
    lv_obj_add_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);

    // Sector delta labels (under LAP number on the left)
    const char* sector_names[3] = {"S1", "S2", "S3"};
    for (int i = 0; i < 3; i++) {
        lbl_sector_deltas[i] = lv_label_create(scr);
        lv_obj_set_style_text_color(lbl_sector_deltas[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl_sector_deltas[i], &share_tech_mono_24, 0);
        lv_obj_set_width(lbl_sector_deltas[i], 120);
        lv_obj_set_style_text_align(lbl_sector_deltas[i], LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(lbl_sector_deltas[i], LV_ALIGN_TOP_LEFT, 8, 42 + (i * 26));
        char buf[16];
        snprintf(buf, sizeof(buf), "%s --.--", sector_names[i]);
        lv_label_set_text(lbl_sector_deltas[i], buf);
        lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Notification label (bottom center, hidden by default)
    lbl_notification = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_notification, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_notification, &share_tech_mono_24, 0);
    lv_obj_set_style_bg_color(lbl_notification, lv_color_make(0, 80, 0), 0);
    lv_obj_set_style_bg_opa(lbl_notification, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(lbl_notification, 8, 0);
    lv_obj_align(lbl_notification, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_flag(lbl_notification, LV_OBJ_FLAG_HIDDEN);

    // GPS Status page labels (5 lines, Share Tech Mono 24pt)
    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        gps_status_labels[i] = lv_label_create(scr);
        lv_obj_set_style_text_color(gps_status_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(gps_status_labels[i], &share_tech_mono_24, 0);
        lv_obj_set_width(gps_status_labels[i], 620);
        lv_obj_set_style_text_align(gps_status_labels[i], LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(gps_status_labels[i], LV_ALIGN_TOP_LEFT, 10, 6 + (i * 32));
        lv_label_set_text(gps_status_labels[i], "");
        lv_obj_add_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Lap completion fullscreen overlay (hidden by default)
    lap_complete_overlay = lv_obj_create(scr);
    lv_obj_set_size(lap_complete_overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(lap_complete_overlay, 0, 0);
    lv_obj_set_style_bg_color(lap_complete_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lap_complete_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(lap_complete_overlay, 0, 0);
    lv_obj_set_style_pad_all(lap_complete_overlay, 0, 0);
    lv_obj_set_style_radius(lap_complete_overlay, 0, 0);
    lv_obj_add_flag(lap_complete_overlay, LV_OBJ_FLAG_HIDDEN);

    // Lap completion time label (centered on overlay)
    lap_complete_time_label = lv_label_create(lap_complete_overlay);
    lv_obj_set_style_text_color(lap_complete_time_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(lap_complete_time_label, &share_tech_mono_72, 0);
    lv_obj_set_width(lap_complete_time_label, 420);
    lv_obj_set_style_text_align(lap_complete_time_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lap_complete_time_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(lap_complete_time_label, "");

    // Lap completion border (for best lap indication)
    lap_complete_border = lv_obj_create(lap_complete_overlay);
    lv_obj_set_size(lap_complete_border, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(lap_complete_border, 0, 0);
    lv_obj_set_style_bg_opa(lap_complete_border, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(lap_complete_border, lv_color_make(20, 170, 20), 0);
    lv_obj_set_style_border_width(lap_complete_border, 6, 0);
    lv_obj_set_style_radius(lap_complete_border, 0, 0);
    lv_obj_set_style_pad_all(lap_complete_border, 0, 0);
    lv_obj_add_flag(lap_complete_border, LV_OBJ_FLAG_HIDDEN);

    // ── Lap Summary overlay (full screen, hidden by default) ──
    s_summaryOverlay = lv_obj_create(scr);
    lv_obj_set_size(s_summaryOverlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(s_summaryOverlay, 0, 0);
    lv_obj_set_style_bg_color(s_summaryOverlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_summaryOverlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_summaryOverlay, 0, 0);
    lv_obj_set_style_pad_all(s_summaryOverlay, 0, 0);
    lv_obj_set_style_radius(s_summaryOverlay, 0, 0);
    lv_obj_add_flag(s_summaryOverlay, LV_OBJ_FLAG_HIDDEN);

    // Create grid cells: row 0 = header (yellow), rows 1..6 = data (white)
    for (int r = 0; r < SUMMARY_ROWS; r++) {
        lv_color_t cellColor = (r == 0) ? lv_color_make(255, 220, 0) : lv_color_white();
        for (int c = 0; c < SUMMARY_TOTAL_COLS; c++) {
            lv_obj_t* lbl = lv_label_create(s_summaryOverlay);
            lv_obj_set_style_text_color(lbl, cellColor, 0);
            lv_obj_set_style_text_font(lbl, &share_tech_mono_24, 0);
            lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_LEFT, 0);
            lv_obj_set_pos(lbl, SUMMARY_COL_X[c], SUMMARY_ROW_Y[r]);
            lv_label_set_text(lbl, "");
            s_summaryCells[r][c] = lbl;
        }
    }

    // ── IMU Status overlay (full screen, hidden by default) ──
    s_imuOverlay = lv_obj_create(scr);
    lv_obj_set_size(s_imuOverlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(s_imuOverlay, 0, 0);
    lv_obj_set_style_bg_color(s_imuOverlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_imuOverlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_imuOverlay, 0, 0);
    lv_obj_set_style_pad_all(s_imuOverlay, 0, 0);
    lv_obj_set_style_radius(s_imuOverlay, 0, 0);
    lv_obj_clear_flag(s_imuOverlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_imuOverlay, LV_OBJ_FLAG_HIDDEN);

    // G-force circle container
    {
        lv_obj_t* gc = lv_obj_create(s_imuOverlay);
        lv_obj_set_size(gc, GF_SIZE, GF_SIZE);
        lv_obj_set_pos(gc, 4, 6);
        lv_obj_set_style_bg_color(gc, lv_color_make(10, 10, 10), 0);
        lv_obj_set_style_bg_opa(gc, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(gc, 0, 0);
        lv_obj_set_style_pad_all(gc, 0, 0);
        lv_obj_set_style_radius(gc, 0, 0);
        lv_obj_clear_flag(gc, LV_OBJ_FLAG_SCROLLABLE);

        // Circle rings drawn as polylines (avoids expensive AA radius masks)
        static constexpr int CIRCLE_SEGS = 48;
        static lv_point_precise_t pts1g[CIRCLE_SEGS + 1];
        static lv_point_precise_t pts05g[CIRCLE_SEGS + 1];
        float cx = GF_SIZE / 2.0f;
        float cy = GF_SIZE / 2.0f;
        for (int i = 0; i <= CIRCLE_SEGS; i++) {
            float a = 6.2831853f * i / CIRCLE_SEGS;
            float ca = cosf(a), sa = sinf(a);
            pts1g[i].x  = (int)(cx + GF_SCALE * ca);
            pts1g[i].y  = (int)(cy + GF_SCALE * sa);
            pts05g[i].x = (int)(cx + (GF_SCALE * 0.5f) * ca);
            pts05g[i].y = (int)(cy + (GF_SCALE * 0.5f) * sa);
        }

        // 1G ring
        lv_obj_t* line1g = lv_line_create(gc);
        lv_line_set_points(line1g, pts1g, CIRCLE_SEGS + 1);
        lv_obj_set_style_line_color(line1g, lv_color_make(80, 80, 80), 0);
        lv_obj_set_style_line_width(line1g, 1, 0);

        // 0.5G ring
        lv_obj_t* line05g = lv_line_create(gc);
        lv_line_set_points(line05g, pts05g, CIRCLE_SEGS + 1);
        lv_obj_set_style_line_color(line05g, lv_color_make(50, 50, 50), 0);
        lv_obj_set_style_line_width(line05g, 1, 0);

        // Crosshairs
        static lv_point_precise_t hpts[] = {{0, GF_SIZE / 2}, {GF_SIZE, GF_SIZE / 2}};
        lv_obj_t* lH = lv_line_create(gc);
        lv_line_set_points(lH, hpts, 2);
        lv_obj_set_style_line_color(lH, lv_color_make(50, 50, 50), 0);
        lv_obj_set_style_line_width(lH, 1, 0);

        static lv_point_precise_t vpts[] = {{GF_SIZE / 2, 0}, {GF_SIZE / 2, GF_SIZE}};
        lv_obj_t* lV = lv_line_create(gc);
        lv_line_set_points(lV, vpts, 2);
        lv_obj_set_style_line_color(lV, lv_color_make(50, 50, 50), 0);
        lv_obj_set_style_line_width(lV, 1, 0);

        // Moving dot (8x8 square — no radius to avoid AA mask cost)
        s_gfDot = lv_obj_create(gc);
        lv_obj_set_size(s_gfDot, 8, 8);
        lv_obj_set_style_radius(s_gfDot, 0, 0);
        lv_obj_set_style_bg_color(s_gfDot, lv_color_make(0, 230, 0), 0);
        lv_obj_set_style_bg_opa(s_gfDot, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(s_gfDot, 0, 0);
        lv_obj_clear_flag(s_gfDot, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(s_gfDot, GF_SIZE / 2 - 4, GF_SIZE / 2 - 4);
    }

    // IMU info labels (5 lines, 24pt)
    for (int i = 0; i < IMU_INFO_LINES; i++) {
        s_imuLabels[i] = lv_label_create(s_imuOverlay);
        lv_obj_set_style_text_font(s_imuLabels[i], &share_tech_mono_24, 0);
        lv_obj_set_style_text_color(s_imuLabels[i],
            (i == 0) ? lv_color_make(255, 220, 0) : lv_color_white(), 0);
        lv_obj_set_pos(s_imuLabels[i], 196, 8 + i * 32);
        lv_label_set_text(s_imuLabels[i], "");
    }

    // Z-axis vertical bar (lightweight replacement for chart)
    {
        s_zBar = lv_obj_create(s_imuOverlay);
        lv_obj_set_size(s_zBar, ZB_WIDTH, ZB_HEIGHT);
        lv_obj_set_pos(s_zBar, 170, 6);
        lv_obj_set_style_bg_color(s_zBar, lv_color_make(15, 15, 15), 0);
        lv_obj_set_style_bg_opa(s_zBar, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(s_zBar, lv_color_make(60, 60, 60), 0);
        lv_obj_set_style_border_width(s_zBar, 1, 0);
        lv_obj_set_style_radius(s_zBar, 0, 0);
        lv_obj_set_style_pad_all(s_zBar, 0, 0);
        lv_obj_clear_flag(s_zBar, LV_OBJ_FLAG_SCROLLABLE);

        // Center line (0G mark)
        static lv_point_precise_t zMidPts[] = {{0, ZB_HEIGHT / 2}, {ZB_WIDTH, ZB_HEIGHT / 2}};
        lv_obj_t* zMidLine = lv_line_create(s_zBar);
        lv_line_set_points(zMidLine, zMidPts, 2);
        lv_obj_set_style_line_color(zMidLine, lv_color_make(80, 80, 80), 0);
        lv_obj_set_style_line_width(zMidLine, 1, 0);

        // Z dot (cyan, 8x8 square)
        s_zDot = lv_obj_create(s_zBar);
        lv_obj_set_size(s_zDot, 10, 8);
        lv_obj_set_style_radius(s_zDot, 0, 0);
        lv_obj_set_style_bg_color(s_zDot, lv_color_make(0, 200, 255), 0);
        lv_obj_set_style_bg_opa(s_zDot, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(s_zDot, 0, 0);
        lv_obj_clear_flag(s_zDot, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(s_zDot, 3, ZB_HEIGHT / 2 - 4);
    }

    lvglUnlock();

    ESP_LOGI(TAG, "UI created (landscape 640x172)");
}

// ============================================================
// UPDATE FUNCTIONS
// ============================================================

void updateLapData(void)
{
    // ─── PRE_TRACK 모드: 속도 + 현재 시각 표시 (세션 시작 전) ───
    if (s_preTrackMode) {
        // GPS 속도 (gApp.currentPoint는 gps_processor에서 GPS 수신 시마다 업데이트)
        int speedKmh = (int)gApp.currentPoint.speedKmh;
        char speedBuf[16];
        snprintf(speedBuf, sizeof(speedBuf), "%d", speedKmh);

        // 하단 텍스트: 트랙 식별됨이면 트랙 이름, 아니면 현재 시각
        char bottomBuf[72] = "";
        uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
        bool showCalibMsg = (gApp.fusionCalibDoneMs > 0 &&
                             (nowMs - gApp.fusionCalibDoneMs) < 3000);

        if (showCalibMsg) {
            snprintf(bottomBuf, sizeof(bottomBuf), "IMU Calibrated");
        } else if (s_preTrackName[0] != '\0') {
            // 트랙 이름 표시
            snprintf(bottomBuf, sizeof(bottomBuf), ">> %s <<", s_preTrackName);
        } else {
            // PRE_TRACK: 현재 시각 (TZ env var 기준 로컬 시각)
            time_t now_t = time(nullptr);
            struct tm tmInfo = {};
            localtime_r(&now_t, &tmInfo);
            snprintf(bottomBuf, sizeof(bottomBuf), "%02d:%02d:%02d",
                     tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec);
        }

        if (lvglLock(10)) {
            if (lbl_delta) {
                // Right-align speed so digit count changes don't shift position
                lv_obj_set_width(lbl_delta, 180);
                lv_obj_set_style_text_align(lbl_delta, LV_TEXT_ALIGN_RIGHT, 0);
                lv_obj_align(lbl_delta, LV_ALIGN_CENTER, -60, 0);
                lv_label_set_text(lbl_delta, speedBuf);
            }
            if (lbl_center_unit) {
                lv_obj_set_style_text_font(lbl_center_unit, &share_tech_mono_32, 0);
                lv_obj_align(lbl_center_unit, LV_ALIGN_CENTER, 91, 12);
                lv_obj_clear_flag(lbl_center_unit, LV_OBJ_FLAG_HIDDEN);
            }
            if (lbl_datetime) {
                lv_label_set_text(lbl_datetime, bottomBuf);
                lv_obj_clear_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
            }
            lvglUnlock();
        }
        lvalid = false;
        return;
    }

    // ═══ Phase 1: Data Preparation (mutex 불필요) ═══
    // 모든 연산/포맷팅을 먼저 수행하여 mutex 점유 시간 최소화

    bool isFirstLap = (tframe.lap <= 1);
    bool hasReference = gApp.hasValidReferenceLap && !isFirstLap;

    // Lap completion overlay 데이터
    bool lapCompleteMode = (lframe.lapCompleteDisplayEndMs > 0 &&
                           (esp_timer_get_time() / 1000) < lframe.lapCompleteDisplayEndMs);
    char lcTimeBuf[16] = "";
    bool lcIsBest = false;
    if (lapCompleteMode) {
        uint32_t ct = lframe.lastCompletedLapMs;
        lcIsBest = (lframe.hasBestLap && ct <= lframe.bestLapMs);
        uint32_t mins = ct / 60000;
        uint32_t secs = (ct / 1000) % 60;
        uint32_t ms = (ct % 1000) / 10;
        snprintf(lcTimeBuf, sizeof(lcTimeBuf), "%lu:%02lu.%02lu", mins, secs, ms);
    }

    // EMA 스무딩
    float rawDelta = lframe.delta / 1000.0f;
    if (cache.delta[0] == '\0') {
        smoothedDelta = rawDelta;
    } else {
        smoothedDelta = DELTA_EMA_ALPHA * rawDelta + (1.0f - DELTA_EMA_ALPHA) * smoothedDelta;
    }

    // 랩타임 & 델타 텍스트 포맷팅
    uint32_t t = lframe.time;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%lu:%02lu.%lu",
             t / 60000, (t / 1000) % 60, (t % 1000) / 100);

    char deltaBuf[16];
    snprintf(deltaBuf, sizeof(deltaBuf), "%+.2f", smoothedDelta);

    // 페이지 설정에 따른 중앙/보조 텍스트 결정
    const PageConfig& pc = getDisplayConfig().pages[s_userPageIndex];
    char speedCenterBuf[16];
    CenterContent effectiveCenter = pc.center;
    if (!hasReference && effectiveCenter == CenterContent::DELTA) {
        effectiveCenter = CenterContent::LAPTIME;  // 레퍼런스 없으면 델타 대신 랩타임
    }
    const char *centerText;
    const char *laptimeText;
    switch (effectiveCenter) {
        case CenterContent::SPEED:
            snprintf(speedCenterBuf, sizeof(speedCenterBuf), "%d", (int)gApp.currentPoint.speedKmh);
            centerText  = speedCenterBuf;
            laptimeText = timeBuf;
            break;
        case CenterContent::LAPTIME:
            centerText  = timeBuf;
            laptimeText = hasReference ? deltaBuf : "Ref Lap";
            break;
        case CenterContent::DELTA:
        default:
            centerText  = deltaBuf;
            laptimeText = timeBuf;
            break;
    }

    // Lap number
    char lapBuf[16];
    snprintf(lapBuf, sizeof(lapBuf), "Lap%02u", (unsigned)tframe.lap);

    // Top 3 laps
    char top3Buf[64] = "";
    int validCount = 0;
    for (int i = 0; i < 3; i++) {
        if (gApp.top3Laps[i].lapNumber > 0 && gApp.top3Laps[i].lapTimeMs < UINT32_MAX) {
            char line[24];
            uint32_t lt = gApp.top3Laps[i].lapTimeMs;
            snprintf(line, sizeof(line), "L%02u %lu:%02lu.%02lu\n",
                     gApp.top3Laps[i].lapNumber,
                     lt / 60000, (lt / 1000) % 60, (lt % 1000) / 10);
            strncat(top3Buf, line, sizeof(top3Buf) - strlen(top3Buf) - 1);
            validCount++;
        }
    }
    if (validCount == 0) {
        snprintf(top3Buf, sizeof(top3Buf), "L-- --:--.--");
    }

    // Speed delta EMA
    if (gApp.currentDelta.hasSpeedDelta) {
        float rawSpeedDeltaKmh = gApp.currentDelta.speedDeltaKmh;
        smoothedSpeedDeltaKmh = SPEED_DELTA_EMA_ALPHA * rawSpeedDeltaKmh
                                + (1.0f - SPEED_DELTA_EMA_ALPHA) * smoothedSpeedDeltaKmh;
    } else {
        smoothedSpeedDeltaKmh *= 0.92f;
        if (fabsf(smoothedSpeedDeltaKmh) < 0.05f) {
            smoothedSpeedDeltaKmh = 0.0f;
        }
    }

    // Bar 치수 계산 (bar 모드에 따라 time 또는 speed 기반)
    const int screen_w = LCD_H_RES;
    const int screen_h = LCD_V_RES;
    int32_t barValue;
    float ratio;
    bool faster, slower;
    switch (pc.bar) {
        case BarMode::TIME: {
            float rangeMs = DELTA_BAR_RANGE_SECONDS * 1000.0f;
            float deltaMs = (float)lframe.delta;
            if (deltaMs >  rangeMs) deltaMs =  rangeMs;
            if (deltaMs < -rangeMs) deltaMs = -rangeMs;
            ratio  = fabsf(deltaMs) / rangeMs;
            faster = hasReference && (lframe.delta < -50);   // 50ms deadzone
            slower = hasReference && (lframe.delta >  50);
            barValue = lframe.delta;
            break;
        }
        case BarMode::SPEED:
        default: {
            ratio  = fabsf(smoothedSpeedDeltaKmh) / SPEED_BAR_RANGE_KMH;
            if (ratio > 1.0f) ratio = 1.0f;
            faster  = hasReference && (smoothedSpeedDeltaKmh >  SPEED_BAR_DEADZONE_KMH);
            slower  = hasReference && (smoothedSpeedDeltaKmh < -SPEED_BAR_DEADZONE_KMH);
            barValue = (int32_t)(smoothedSpeedDeltaKmh * 100.0f);
            break;
        }
    }
    int fill = (int)(ratio * (float)screen_w);
    if (fill < 0) fill = 0;
    if (fill > screen_w) fill = screen_w;
    bool showBarValue = (pc.bar != BarMode::NONE) && hasReference && (faster || slower);

    // Bar value 텍스트 (speed: km/h, time: seconds)
    char speedBuf[24] = "";
    if (showBarValue) {
        if (pc.bar == BarMode::SPEED) {
            snprintf(speedBuf, sizeof(speedBuf), "%+.1f", smoothedSpeedDeltaKmh);
        } else {
            float deltaSec = lframe.delta / 1000.0f;
            snprintf(speedBuf, sizeof(speedBuf), "%+.2f", deltaSec);
        }
    }

    // Sector 데이터 준비
    const CurrentSectorTiming& sectorTiming = getCurrentSectorTiming();
    bool showSectors = (sectorTiming.totalSectors > 0 && hasReference && pc.showSectors);
    int totalSectors = sectorTiming.totalSectors;

    struct { char text[32]; bool show; } sectorData[3] = {};
    if (showSectors) {
        const char* sector_names[3] = {"S1", "S2", "S3"};
        float totalDeltaSec = gApp.currentDelta.deltaSeconds;
        float completedCumulativeDelta = 0.0f;
        if (sectorTiming.completedCount > 0) {
            completedCumulativeDelta = sectorTiming.cumulativeDeltaAtExit[sectorTiming.completedCount - 1];
        }
        for (int i = 0; i < 3 && i < totalSectors; i++) {
            float delta = 0.0f;
            bool isCurrent = (i == sectorTiming.currentSector);
            if (sectorTiming.sectorCompleted[i]) {
                delta = sectorTiming.sectorDeltas[i];
            } else if (isCurrent) {
                delta = totalDeltaSec - completedCumulativeDelta;
            }
            sectorData[i].show = sectorTiming.sectorCompleted[i] || isCurrent;
            if (sectorData[i].show) {
                const char* sign = (delta >= 0) ? "+" : "";
                snprintf(sectorData[i].text, sizeof(sectorData[i].text),
                         "%s %s%.2f", sector_names[i], sign, delta);
            }
        }
    }

    // ═══ Phase 2: LVGL Widget Updates (mutex 필요) ═══
    if (!lvglLock(10)) return;

    // Restore pretrack overrides (right-aligned speed, smaller km/h font)
    if (lbl_delta) {
        lv_obj_set_width(lbl_delta, 420);
        lv_obj_set_style_text_align(lbl_delta, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(lbl_delta, LV_ALIGN_CENTER, 0, 0);
    }
    if (lbl_center_unit) {
        lv_obj_set_style_text_font(lbl_center_unit, &share_tech_mono_56, 0);
        lv_obj_align(lbl_center_unit, LV_ALIGN_CENTER, 120, 8);
    }

    // Defensive: ensure GPS status labels hidden while on main page
    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        if (gps_status_labels[i]) lv_obj_add_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Lap completion fullscreen overlay
    if (lapCompleteMode && lap_complete_overlay) {
        lv_label_set_text(lap_complete_time_label, lcTimeBuf);
        if (lcIsBest) lv_obj_clear_flag(lap_complete_border, LV_OBJ_FLAG_HIDDEN);
        else          lv_obj_add_flag(lap_complete_border, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lap_complete_overlay, LV_OBJ_FLAG_HIDDEN);
        lvglUnlock();
        lvalid = false;
        return;
    } else if (lap_complete_overlay) {
        lv_obj_add_flag(lap_complete_overlay, LV_OBJ_FLAG_HIDDEN);
    }

    // 우상단 라벨 + 레퍼런스 유무에 따른 표시
    if (hasReference) {
        if (strcmp(laptimeText, cache.lapTime) != 0) {
            lv_label_set_text(lbl_laptime, laptimeText);
            strncpy(cache.lapTime, laptimeText, sizeof(cache.lapTime) - 1);
        }
        lv_obj_clear_flag(lbl_laptime, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);
    } else {
        if (strcmp("Ref Lap", cache.lapTime) != 0) {
            lv_label_set_text(lbl_laptime, "Ref Lap");
            strncpy(cache.lapTime, "Ref Lap", sizeof(cache.lapTime) - 1);
        }
        lv_obj_clear_flag(lbl_laptime, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);
    }

    // Lap number
    if (strcmp(lapBuf, cache.lapNum) != 0) {
        lv_label_set_text(lbl_lapnum, lapBuf);
        strcpy(cache.lapNum, lapBuf);
    }
    if (!pc.showLapNumber) lv_obj_add_flag(lbl_lapnum, LV_OBJ_FLAG_HIDDEN);

    // Top 3 laps
    if (strcmp(top3Buf, cache.best) != 0) {
        lv_label_set_text(lbl_best, top3Buf);
        strcpy(cache.best, top3Buf);
    }
    if (!pc.showBestLaps) lv_obj_add_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);

    // Delta bars (NONE 모드이거나 레퍼런스 없으면 숨김)
    bool showBar = (pc.bar != BarMode::NONE) && hasReference;
    if (!showBar) {
        lv_obj_set_size(bar_up, 0, screen_h);
        lv_obj_set_size(bar_down, 0, screen_h);
        lv_obj_set_pos(bar_up, 0, 0);
        lv_obj_set_pos(bar_down, screen_w, 0);
    } else if (barValue != cache.lastBarValue && bar_bg && bar_up && bar_down) {
        if (faster) {
            lv_obj_set_size(bar_up, fill, screen_h);
            lv_obj_set_pos(bar_up, 0, 0);
            lv_obj_set_size(bar_down, 0, screen_h);
            lv_obj_set_pos(bar_down, screen_w, 0);
        } else if (slower) {
            lv_obj_set_size(bar_down, fill, screen_h);
            lv_obj_set_pos(bar_down, screen_w - fill, 0);
            lv_obj_set_size(bar_up, 0, screen_h);
            lv_obj_set_pos(bar_up, 0, 0);
        } else {
            lv_obj_set_size(bar_up, 0, screen_h);
            lv_obj_set_size(bar_down, 0, screen_h);
            lv_obj_set_pos(bar_up, 0, 0);
            lv_obj_set_pos(bar_down, screen_w, 0);
        }
        cache.lastBarValue = barValue;
    }

    // Speed delta text (위치 계산은 lv_obj_get_width 필요하므로 mutex 내부)
    if (lbl_speed_delta && showBarValue) {
        if (strcmp(speedBuf, cache.speedDelta) != 0) {
            lv_label_set_text(lbl_speed_delta, speedBuf);
            strcpy(cache.speedDelta, speedBuf);
        }
        lv_obj_clear_flag(lbl_speed_delta, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_text_color(lbl_speed_delta, lv_color_white(), 0);
        lv_obj_update_layout(lbl_speed_delta);
        int txt_w = lv_obj_get_width(lbl_speed_delta);
        int txt_h = lv_obj_get_height(lbl_speed_delta);
        int y = screen_h - txt_h - 4;
        int x = 0;
        if (faster) {
            x = fill - txt_w - 8;
            if (x < 4) x = 4;
        } else {
            int edge_x = screen_w - fill;
            x = edge_x + 8;
            if (x + txt_w > screen_w - 4) x = screen_w - txt_w - 4;
        }
        lv_obj_set_pos(lbl_speed_delta, x, y);
    } else if (lbl_speed_delta) {
        lv_obj_add_flag(lbl_speed_delta, LV_OBJ_FLAG_HIDDEN);
        cache.speedDelta[0] = '\0';
    }

    // 중앙 라벨
    if (strcmp(centerText, cache.delta) != 0) {
        lv_obj_set_style_text_color(lbl_delta, lv_color_white(), 0);
        lv_label_set_text(lbl_delta, centerText);
        strncpy(cache.delta, centerText, sizeof(cache.delta) - 1);
    }
    lv_obj_clear_flag(lbl_delta, LV_OBJ_FLAG_HIDDEN);

    // km/h unit label: only in speed mode
    if (lbl_center_unit) {
        if (effectiveCenter == CenterContent::SPEED)
            lv_obj_clear_flag(lbl_center_unit, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(lbl_center_unit, LV_OBJ_FLAG_HIDDEN);
    }

    // Sector deltas
    if (showSectors) {
        for (int i = 0; i < 3 && i < totalSectors; i++) {
            if (sectorData[i].show) {
                lv_obj_set_style_text_color(lbl_sector_deltas[i], lv_color_white(), 0);
                lv_label_set_text(lbl_sector_deltas[i], sectorData[i].text);
                lv_obj_clear_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
        for (int i = totalSectors; i < 3; i++) {
            lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        for (int i = 0; i < 3; i++) {
            lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // 날짜/시간 표시 (시간 설정 완료 시)
    if (lbl_datetime && tframe.hours != 0xFF) {
        static char cache_datetime[24] = "";
        char dtBuf[24];
        snprintf(dtBuf, sizeof(dtBuf), "%04u/%02u/%02u %02u:%02u",
                 tframe.year, tframe.month, tframe.date,
                 tframe.hours, tframe.minutes);
        if (strcmp(dtBuf, cache_datetime) != 0) {
            lv_label_set_text(lbl_datetime, dtBuf);
            strncpy(cache_datetime, dtBuf, sizeof(cache_datetime) - 1);
        }
        lv_obj_clear_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
    } else if (lbl_datetime) {
        lv_obj_add_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
    }
    if (lbl_datetime && !pc.showDatetime) lv_obj_add_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);

    lvglUnlock();
    lvalid = false;
}

void updateGpsData(void)
{
    gvalid = false;
}

void updateTimeData(void)
{
    // Time label is intentionally hidden in current UI layout.
    tvalid = false;
}

// ============================================================
// GPS SIGNAL STATUS
// ============================================================

static bool s_gpsSignalLost = false;

void setGpsSignalLost(bool lost)
{
    s_gpsSignalLost = lost;
}

bool isGpsSignalLost(void)
{
    return s_gpsSignalLost;
}

// ============================================================
// NOTIFICATIONS
// ============================================================

static uint64_t s_notificationEndMs = 0;
static bool s_notificationActive = false;

void showNotification(const char* message, uint16_t durationMs)
{
    if (!lvglLock(10)) return;

    lv_label_set_text(lbl_notification, message);
    lv_obj_clear_flag(lbl_notification, LV_OBJ_FLAG_HIDDEN);

    s_notificationEndMs = (esp_timer_get_time() / 1000) + durationMs;
    s_notificationActive = true;

    lvglUnlock();
}



static void updateNotification(void)
{
    if (s_notificationActive) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now >= s_notificationEndMs) {
            if (lvglLock(10)) {
                lv_obj_add_flag(lbl_notification, LV_OBJ_FLAG_HIDDEN);
                lvglUnlock();
            }
            s_notificationActive = false;
            cache.lapTime[0] = '\0';  // Force refresh
        }
    }
}

// ============================================================
// GESTURE DETECTION
// ============================================================

// 제스처 감지 (좌표 기반, HAL readTouchXY() API 활용)
// 맨하탄 거리로 스와이프 판정, |dx| vs |dy| 로 방향 결정
enum GestureResult : uint8_t {
    GESTURE_NONE,
    GESTURE_TAP,
    GESTURE_SWIPE_LEFT,
    GESTURE_SWIPE_RIGHT,
    GESTURE_SWIPE_UP,
    GESTURE_SWIPE_DOWN,
};

static bool    s_gestureTracking = false;
static int16_t s_gestureStartX   = 0;
static int16_t s_gestureStartY   = 0;
static int16_t s_gestureLastX    = 0;
static int16_t s_gestureLastY    = 0;


// HAL readTouchXY() 기반 제스처 감지 (매 프레임 호출)
static GestureResult detectGesture(void)
{
    int16_t cx, cy;
    bool pressed = readTouchXY(&cx, &cy);

    if (pressed) {
        if (!s_gestureTracking) {
            // 터치 시작
            s_gestureTracking = true;
            s_gestureStartX = cx;
            s_gestureStartY = cy;
            s_gestureLastX = cx;
            s_gestureLastY = cy;
        } else {
            // 터치 유지 중 — 좌표 갱신
            s_gestureLastX = cx;
            s_gestureLastY = cy;
        }
        return GESTURE_NONE;  // 아직 손가락 누르고 있음
    }

    // 손가락 뗌 → 제스처 판정
    if (!s_gestureTracking) return GESTURE_NONE;
    s_gestureTracking = false;

    int16_t dx = s_gestureLastX - s_gestureStartX;
    int16_t dy = s_gestureLastY - s_gestureStartY;
    int16_t absDx = (dx >= 0) ? dx : -dx;
    int16_t absDy = (dy >= 0) ? dy : -dy;
    int16_t manhattan = absDx + absDy;

    if (manhattan < TOUCH_SWIPE_THRESHOLD_PX) {
        return GESTURE_TAP;
    }

    // Shared touch coordinates are the same raw axis mapping used by the
    // Waveshare example. For 270° display rotation (180° flip from 90°),
    // invert gesture directions to match what the user sees on screen.
    const bool flipped180 = (DISPLAY_ROTATION_DEG == 270);
    if (absDy >= absDx) {
        if (!flipped180) return (dy > 0) ? GESTURE_SWIPE_RIGHT : GESTURE_SWIPE_LEFT;
        return (dy > 0) ? GESTURE_SWIPE_LEFT : GESTURE_SWIPE_RIGHT;
    }
    if (!flipped180) return (dx > 0) ? GESTURE_SWIPE_DOWN : GESTURE_SWIPE_UP;
    return (dx > 0) ? GESTURE_SWIPE_UP : GESTURE_SWIPE_DOWN;
}

// ============================================================
// STARTUP SCREEN
// ============================================================

void createStartupScreen(void)
{
    if (!lvglLock(-1)) return;

    lv_obj_t *scr = lv_scr_act();

    // 전체 화면 오버레이 (모든 UI 위에 렌더링)
    startup_overlay = lv_obj_create(scr);
    lv_obj_set_size(startup_overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(startup_overlay, 0, 0);
    lv_obj_set_style_bg_color(startup_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(startup_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(startup_overlay, 0, 0);
    lv_obj_set_style_pad_all(startup_overlay, 0, 0);
    lv_obj_set_style_radius(startup_overlay, 0, 0);

    // 저전력 경고 텍스트 (startup 화면)
    lbl_bat_low_startup = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(lbl_bat_low_startup, lv_color_hex(0x551111), 0);
    lv_obj_set_style_text_font(lbl_bat_low_startup, &lv_font_montserrat_14, 0);
    lv_label_set_text(lbl_bat_low_startup, "BAT LOW");
    lv_obj_align(lbl_bat_low_startup, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(lbl_bat_low_startup, LV_OBJ_FLAG_HIDDEN);

    // 배터리 아이콘 — 오른쪽 상단
    bat_icon_body = lv_obj_create(startup_overlay);
    lv_obj_set_size(bat_icon_body, 36, 16);
    lv_obj_set_pos(bat_icon_body, 594, 6);
    lv_obj_set_style_bg_opa(bat_icon_body, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(bat_icon_body, lv_color_white(), 0);
    lv_obj_set_style_border_width(bat_icon_body, 2, 0);
    lv_obj_set_style_radius(bat_icon_body, 3, 0);
    lv_obj_set_style_pad_all(bat_icon_body, 0, 0);
    lv_obj_clear_flag(bat_icon_body, LV_OBJ_FLAG_SCROLLABLE);

    bat_icon_tip = lv_obj_create(startup_overlay);
    lv_obj_set_size(bat_icon_tip, 3, 8);
    lv_obj_set_pos(bat_icon_tip, 630, 10);
    lv_obj_set_style_bg_color(bat_icon_tip, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(bat_icon_tip, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bat_icon_tip, 0, 0);
    lv_obj_set_style_radius(bat_icon_tip, 1, 0);
    lv_obj_clear_flag(bat_icon_tip, LV_OBJ_FLAG_SCROLLABLE);

    bat_icon_fill = lv_obj_create(startup_overlay);
    lv_obj_set_size(bat_icon_fill, 0, 10);
    lv_obj_set_pos(bat_icon_fill, 597, 9);
    lv_obj_set_style_bg_color(bat_icon_fill, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(bat_icon_fill, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bat_icon_fill, 0, 0);
    lv_obj_set_style_radius(bat_icon_fill, 1, 0);
    lv_obj_clear_flag(bat_icon_fill, LV_OBJ_FLAG_SCROLLABLE);

    // 프로젝트 타이틀: "GPS LAPTIMER" (56pt, 흰색, 중앙)
    startup_title_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_title_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(startup_title_label, &share_tech_mono_56, 0);
    lv_obj_set_width(startup_title_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_title_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_title_label, LV_ALIGN_TOP_MID, 0, 20);
    lv_label_set_text(startup_title_label, "GPS LAPTIMER");

    // 버전: "v0.4.0" (24pt, 회색, 중앙)
    startup_version_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_version_label, lv_color_make(0x80, 0x80, 0x80), 0);
    lv_obj_set_style_text_font(startup_version_label, &share_tech_mono_24, 0);
    lv_obj_set_width(startup_version_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_version_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_version_label, LV_ALIGN_TOP_MID, 0, 80);
    lv_label_set_text(startup_version_label, APP_VERSION);

    // GPS/모드 상태 (32pt, 중앙, 동적 색상)
    startup_status_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_status_label, lv_color_make(0xFF, 0xCC, 0x00), 0);
    lv_obj_set_style_text_font(startup_status_label, &share_tech_mono_32, 0);
    lv_obj_set_width(startup_status_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_status_label, LV_ALIGN_TOP_MID, 0, 120);
    lv_label_set_text(startup_status_label, "");

    // 모드 선택 힌트 (14pt, 회색, 중앙) — MODE_SELECT 상태에서만 표시
    startup_hint_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_hint_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(startup_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(startup_hint_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_hint_label, LV_ALIGN_TOP_MID, 0, 155);
    lv_label_set_text(startup_hint_label, "");
    lv_obj_add_flag(startup_hint_label, LV_OBJ_FLAG_HIDDEN);

    // ── 전화번호 플레이트 (화면: PHONE_PLATE) ──
    phone_overlay = lv_obj_create(scr);
    lv_obj_set_size(phone_overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(phone_overlay, 0, 0);
    lv_obj_set_style_bg_color(phone_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(phone_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(phone_overlay, 0, 0);
    lv_obj_set_style_pad_all(phone_overlay, 0, 0);
    lv_obj_set_style_radius(phone_overlay, 0, 0);

    // 저전력 경고 텍스트 (phone 화면)
    lbl_bat_low_phone = lv_label_create(phone_overlay);
    lv_obj_set_style_text_color(lbl_bat_low_phone, lv_color_hex(0x551111), 0);
    lv_obj_set_style_text_font(lbl_bat_low_phone, &lv_font_montserrat_14, 0);
    lv_label_set_text(lbl_bat_low_phone, "BAT LOW");
    lv_obj_align(lbl_bat_low_phone, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(lbl_bat_low_phone, LV_OBJ_FLAG_HIDDEN);

    phone_number_label = lv_label_create(phone_overlay);
    lv_obj_set_style_text_color(phone_number_label, lv_color_make(0xFF, 0xCC, 0x00), 0);
    lv_obj_set_style_text_font(phone_number_label, &share_tech_mono_56, 0);
    lv_obj_set_width(phone_number_label, LCD_H_RES);
    lv_obj_set_style_text_align(phone_number_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(phone_number_label, LV_ALIGN_CENTER, 0, -10);
    lv_label_set_text(phone_number_label, "NO NUMBER SET");

    phone_hint_label = lv_label_create(phone_overlay);
    lv_obj_set_style_text_color(phone_hint_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(phone_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(phone_hint_label, LCD_H_RES);
    lv_obj_set_style_text_align(phone_hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(phone_hint_label, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_label_set_text(phone_hint_label, "swipe: back");

    lv_obj_add_flag(phone_overlay, LV_OBJ_FLAG_HIDDEN);  // 초기: 숨김

    // 배터리 상태에 따라 BAT LOW 표시
    if (gApp.batteryPercent >= 0.0f && gApp.batteryPercent < 20.0f) {
        if (lbl_bat_low_startup) lv_obj_clear_flag(lbl_bat_low_startup, LV_OBJ_FLAG_HIDDEN);
        if (lbl_bat_low_phone) lv_obj_clear_flag(lbl_bat_low_phone, LV_OBJ_FLAG_HIDDEN);
    }

    lvglUnlock();

    ESP_LOGI(TAG, "Startup screen created");
}

// (updateStartupScreen, dismissStartupScreen, isStartupScreenActive,
//  isGpsStatusOnlyMode removed — replaced by Page system)

// ============================================================
// GPS STATUS PAGE
// ============================================================

static char s_gpsStatusCache[GPS_STATUS_LINE_COUNT][GPS_STATUS_LINE_WIDTH] = {};

/**
 * @brief GPS Status 페이지 데이터 갱신
 */
static void updateGpsStatusPage(void) {
    UBloxData ubx = getUBloxData();
    UBloxStats stats = getUBloxStats();

    // Fix type 문자열
    const char* fixStr = ubx.fixType == 0 ? "No fix" :
                         ubx.fixType == 1 ? "DR" :
                         ubx.fixType == 2 ? "2D" :
                         ubx.fixType == 3 ? "3D" :
                         ubx.fixType == 4 ? "3D+DR" : "?";

    // UART 건강도
    uint32_t totalMsgs = stats.checksumOk + stats.checksumFail;
    int healthPct = totalMsgs > 0 ? (int)(stats.checksumOk * 100 / totalMsgs) : 0;

    // 부팅 후 경과 시간 (uptime)
    int64_t uptimeUs = esp_timer_get_time();
    int uptimeSec = (int)(uptimeUs / 1000000);
    int upH = uptimeSec / 3600;
    int upM = (uptimeSec % 3600) / 60;
    int upS = uptimeSec % 60;

    // 배터리 표시
    int batPct = (gApp.batteryPercent >= 0.0f) ? (int)(gApp.batteryPercent + 0.5f) : 0;

    char lines[GPS_STATUS_LINE_COUNT][GPS_STATUS_LINE_WIDTH] = {};

    // Line 1: header + battery + uptime
    snprintf(lines[0], sizeof(lines[0]),
             "GPS STATUS  BAT:%d%% %.2fV  UPTIME:%02d:%02d:%02d",
             batPct, gApp.batteryVoltage, upH, upM, upS);

    // Line 2: fix + satellite summary (used/total)
    NavSatData sat = getNavSatData();
    int satUsedTotal = sat.usedGps + sat.usedGlo + sat.usedGal + sat.usedBds;
    int satVisibleTotal = sat.numSvs;
    if (sat.valid) {
        snprintf(lines[1], sizeof(lines[1]),
                 "FIX:%s  SAT USED/TOTAL:%d/%d  HDOP:%.1f",
                 fixStr, satUsedTotal, satVisibleTotal, ubx.hdop);
    } else {
        snprintf(lines[1], sizeof(lines[1]),
                 "FIX:%s  SAT USED/TOTAL:%d/%d  HDOP:%.1f",
                 fixStr, ubx.satellites, ubx.satellites, ubx.hdop);
    }

    // Line 3: constellation details (used/total)
    if (sat.valid) {
        snprintf(lines[2], sizeof(lines[2]),
                 "GPS:%d/%d  GLO:%d/%d  GAL:%d/%d  BDS:%d/%d",
                 sat.usedGps, sat.visGps, sat.usedGlo, sat.visGlo,
                 sat.usedGal, sat.visGal, sat.usedBds, sat.visBds);
    } else {
        snprintf(lines[2], sizeof(lines[2]),
                 "GPS:--/--  GLO:--/--  GAL:--/--  BDS:--/--");
    }

    // Line 4: coordinates + speed + altitude
    if (ubx.fixType >= 2) {
        char latDir = ubx.lat >= 0 ? 'N' : 'S';
        char lonDir = ubx.lon >= 0 ? 'E' : 'W';
        snprintf(lines[3], sizeof(lines[3]),
                 "LAT:%c%.6f LON:%c%.6f SPD:%.1f ALT:%.1f",
                 latDir, fabs(ubx.lat), lonDir, fabs(ubx.lon), ubx.speedKmh, ubx.altitudeM);
    } else {
        snprintf(lines[3], sizeof(lines[3]),
                 "LAT:--.------ LON:--.------ SPD:--.- ALT:--.-");
    }

    // Line 5: rate + UART health + UTC
    if (ubx.timeValid) {
        snprintf(lines[4], sizeof(lines[4]),
                 "RATE:%.1fHz  UART:%d%%  UTC:%02u:%02u:%02u",
                 stats.measuredHz, healthPct, ubx.hour, ubx.minute, ubx.second);
    } else {
        snprintf(lines[4], sizeof(lines[4]),
                 "RATE:%.1fHz  UART:%d%%  UTC:--:--:--",
                 stats.measuredHz, healthPct);
    }

    // 캐시 비교 후 변경 시에만 LVGL 업데이트
    if (!lvglLock(10)) return;

    // Defensive: ensure main UI stays hidden while on GPS Status page
    lv_obj_t *hideElements[] = {bar_bg, lbl_delta, lbl_datetime, lbl_lapnum, lbl_laptime, lbl_best, lbl_speed_delta, lap_complete_overlay};
    for (auto el : hideElements) {
        if (el) lv_obj_add_flag(el, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < 3; i++) {
        if (lbl_sector_deltas[i]) lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Per-line colors for readability
    //  0: header/battery/uptime — yellow
    //  1: fix/sat summary       — green(3D), yellow(2D), red(no fix)
    //  2: constellation detail   — cyan
    //  3: coordinates/speed      — white
    //  4: rate/UART/UTC          — grey
    lv_color_t lineColors[GPS_STATUS_LINE_COUNT];
    lineColors[0] = lv_color_make(0xFF, 0xCC, 0x00);  // yellow
    if (ubx.fixType >= 3)
        lineColors[1] = lv_color_make(0x20, 0xDD, 0x20);  // green
    else if (ubx.fixType == 2)
        lineColors[1] = lv_color_make(0xFF, 0xCC, 0x00);  // yellow
    else
        lineColors[1] = lv_color_make(0xFF, 0x44, 0x44);  // red
    lineColors[2] = lv_color_make(0x00, 0xCC, 0xDD);  // cyan
    lineColors[3] = lv_color_white();
    lineColors[4] = lv_color_make(0x99, 0x99, 0x99);  // grey

    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        if (strcmp(lines[i], s_gpsStatusCache[i]) != 0) {
            lv_label_set_text(gps_status_labels[i], lines[i]);
            strncpy(s_gpsStatusCache[i], lines[i], sizeof(s_gpsStatusCache[i]) - 1);
        }
        lv_obj_set_style_text_color(gps_status_labels[i], lineColors[i], 0);
    }

    lvglUnlock();
}

// ============================================================
// DELTA HISTORY
// ============================================================

void resetDeltaHistory(void)
{
    smoothedDelta = 0.0f;
    smoothedSpeedDeltaKmh = 0.0f;
    // Clear cache first, then set lastBarValue to force redraw
    memset(&cache, 0, sizeof(cache));
    cache.lastBarValue = -9999;
}

// ============================================================
// DISPLAY TEST
// ============================================================

void displayTest(void)
{
    ESP_LOGI(TAG, "=== DISPLAY TEST ===");
    ESP_LOGI(TAG, "Display size: %dx%d (landscape)", LCD_H_RES, LCD_V_RES);

    if (!lvglLock(-1)) return;

    lv_obj_t *scr = lv_scr_act();

    // Clear screen
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // RGB test bars (horizontal, spanning full width)
    const int bar_w = LCD_H_RES / 3;
    const int bar_h = LCD_V_RES / 2;

    lv_obj_t *red_bar = lv_obj_create(scr);
    lv_obj_set_size(red_bar, bar_w, bar_h);
    lv_obj_set_pos(red_bar, 0, 0);
    lv_obj_set_style_bg_color(red_bar, lv_color_make(170, 20, 20), 0);
    lv_obj_set_style_border_width(red_bar, 0, 0);

    lv_obj_t *green_bar = lv_obj_create(scr);
    lv_obj_set_size(green_bar, bar_w, bar_h);
    lv_obj_set_pos(green_bar, bar_w, 0);
    lv_obj_set_style_bg_color(green_bar, lv_color_make(20, 170, 20), 0);
    lv_obj_set_style_border_width(green_bar, 0, 0);

    lv_obj_t *blue_bar = lv_obj_create(scr);
    lv_obj_set_size(blue_bar, bar_w, bar_h);
    lv_obj_set_pos(blue_bar, bar_w * 2, 0);
    lv_obj_set_style_bg_color(blue_bar, lv_color_make(0, 0, 255), 0);
    lv_obj_set_style_border_width(blue_bar, 0, 0);

    // Test label
    lv_obj_t *lbl = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl, &share_tech_mono_24, 0);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_label_set_text(lbl, "LANDSCAPE 640x172 OK");

    lvglUnlock();

    ESP_LOGI(TAG, "Test pattern drawn. Check display!");
}

// ============================================================
// DISPLAY WIDGETS API (for Page system)
// ============================================================

Gesture pollGesture(void)
{
    GestureResult gr = detectGesture();
    return static_cast<Gesture>(gr);  // enum values match by design
}

void updateNotificationDisplay(void) { updateNotification(); }

void updateBatteryWarning(void)
{
    int8_t lowBat = (gApp.batteryPercent >= 0.0f && gApp.batteryPercent < 20.0f) ? 1 : 0;
    if (lowBat != cache.lastLowBat) {
        if (lvglLock(10)) {
            lv_obj_t *labels[] = {lbl_bat_low, lbl_bat_low_startup, lbl_bat_low_phone};
            for (int i = 0; i < 3; i++) {
                if (!labels[i]) continue;
                if (lowBat) lv_obj_clear_flag(labels[i], LV_OBJ_FLAG_HIDDEN);
                else        lv_obj_add_flag(labels[i], LV_OBJ_FLAG_HIDDEN);
            }
            lvglUnlock();
            cache.lastLowBat = lowBat;
        }
    }

    // 배터리 아이콘 채움 바 업데이트
    if (bat_icon_fill) {
        int8_t pct = (gApp.batteryPercent < 0.0f) ? 0
                   : (gApp.batteryPercent > 100.0f) ? 100
                   : (int8_t)gApp.batteryPercent;
        if (pct != cache.lastBatIconPct) {
            int fillW = (int)(pct * 30 / 100);
            if (fillW < 0) fillW = 0;
            if (lvglLock(10)) {
                lv_obj_set_width(bat_icon_fill, fillW);
                lvglUnlock();
            }
            cache.lastBatIconPct = pct;
        }
    }
}

// ── Widget getters ──
lv_obj_t* getStartupOverlay(void)      { return startup_overlay; }
lv_obj_t* getStartupTitleLabel(void)    { return startup_title_label; }
lv_obj_t* getStartupVersionLabel(void)  { return startup_version_label; }
lv_obj_t* getStartupStatusLabel(void)   { return startup_status_label; }
lv_obj_t* getStartupHintLabel(void)     { return startup_hint_label; }

lv_obj_t* getPhoneOverlay(void)         { return phone_overlay; }
lv_obj_t* getPhoneNumberLabel(void)     { return phone_number_label; }
lv_obj_t* getPhoneHintLabel(void)       { return phone_hint_label; }

lv_obj_t* getDeltaLabel(void)           { return lbl_delta; }
lv_obj_t* getSpeedDeltaLabel(void)      { return lbl_speed_delta; }
lv_obj_t* getDatetimeLabel(void)        { return lbl_datetime; }
lv_obj_t* getLapnumLabel(void)          { return lbl_lapnum; }
lv_obj_t* getLaptimeLabel(void)         { return lbl_laptime; }
lv_obj_t* getBestLabel(void)            { return lbl_best; }
lv_obj_t* getBarBg(void)               { return bar_bg; }
lv_obj_t* getBarUp(void)               { return bar_up; }
lv_obj_t* getBarDown(void)             { return bar_down; }
lv_obj_t* getNotificationLabel(void)    { return lbl_notification; }

lv_obj_t* getSectorDeltaLabel(int index) {
    if (index < 0 || index >= 3) return NULL;
    return lbl_sector_deltas[index];
}

lv_obj_t* getGpsStatusLabel(int line) {
    if (line < 0 || line >= GPS_STATUS_LINE_COUNT) return NULL;
    return gps_status_labels[line];
}

lv_obj_t* getLapCompleteOverlay(void)    { return lap_complete_overlay; }
lv_obj_t* getLapCompleteTimeLabel(void)  { return lap_complete_time_label; }
lv_obj_t* getLapCompleteBorder(void)     { return lap_complete_border; }

lv_obj_t* getBatLowLabel(void)           { return lbl_bat_low; }
lv_obj_t* getBatLowStartupLabel(void)    { return lbl_bat_low_startup; }
lv_obj_t* getBatLowPhoneLabel(void)      { return lbl_bat_low_phone; }

// ============================================================
// DISPLAY HELPER FUNCTIONS (for Page system)
// ============================================================

void applyPageVisibilityForPage(PageId pageId)
{
    if (!lvglLock(100)) return;

    // Hide startup overlay when entering any main display page
    if (startup_overlay) lv_obj_add_flag(startup_overlay, LV_OBJ_FLAG_HIDDEN);
    if (phone_overlay) lv_obj_add_flag(phone_overlay, LV_OBJ_FLAG_HIDDEN);

    bool isGpsPage   = (pageId == PageId::GPS_STATUS || pageId == PageId::STORAGE_TEST);
    bool isPreTrack  = (pageId == PageId::PRE_TRACK);
    bool isSummary   = (pageId == PageId::LAP_SUMMARY);
    bool isImuPage   = (pageId == PageId::IMU_STATUS);

    // Lap summary overlay: show only on LAP_SUMMARY page
    if (s_summaryOverlay) {
        if (isSummary) lv_obj_clear_flag(s_summaryOverlay, LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_add_flag(s_summaryOverlay, LV_OBJ_FLAG_HIDDEN);
    }

    // IMU overlay: show only on IMU_STATUS page
    if (s_imuOverlay) {
        if (isImuPage) lv_obj_clear_flag(s_imuOverlay, LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_add_flag(s_imuOverlay, LV_OBJ_FLAG_HIDDEN);
    }

    // On fullscreen overlay pages, hide all main UI elements and return early
    if (isSummary || isImuPage) {
        lv_obj_t *allElements[] = {
            bar_bg, lbl_delta, lbl_lapnum, lbl_laptime, lbl_best,
            lbl_speed_delta, lbl_datetime, lbl_notification, lbl_center_unit
        };
        for (auto el : allElements) {
            if (el) lv_obj_add_flag(el, LV_OBJ_FLAG_HIDDEN);
        }
        for (int i = 0; i < 3; i++) {
            if (lbl_sector_deltas[i]) lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
        }
        for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
            if (gps_status_labels[i]) lv_obj_add_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
        lvglUnlock();
        memset(s_gpsStatusCache, 0, sizeof(s_gpsStatusCache));
        memset(&cache, 0, sizeof(cache));
        cache.lastBarValue = -9999;
        cache.lastLowBat = -1;
        return;
    }

    // Main UI elements: hidden on GPS_STATUS and PRE_TRACK (except lbl_delta for speed)
    lv_obj_t *mainElements[] = {
        bar_bg, lbl_delta, lbl_lapnum, lbl_laptime, lbl_best, lbl_speed_delta
    };
    for (auto el : mainElements) {
        if (!el) continue;
        if (isGpsPage) {
            lv_obj_add_flag(el, LV_OBJ_FLAG_HIDDEN);
        } else if (isPreTrack) {
            if (el == lbl_delta) lv_obj_clear_flag(el, LV_OBJ_FLAG_HIDDEN);
            else                 lv_obj_add_flag(el, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(el, LV_OBJ_FLAG_HIDDEN);
        }
    }
    for (int i = 0; i < 3; i++) {
        if (!lbl_sector_deltas[i]) continue;
        if (isGpsPage || isPreTrack) {
            lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // km/h unit label: show on PRE_TRACK (always speed), hide elsewhere
    // (SESSION update will show/hide based on effectiveCenter == SPEED)
    if (lbl_center_unit) {
        if (isPreTrack) lv_obj_clear_flag(lbl_center_unit, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_add_flag(lbl_center_unit, LV_OBJ_FLAG_HIDDEN);
    }

    // GPS Status labels: only visible on GPS_STATUS page
    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        if (!gps_status_labels[i]) continue;
        if (isGpsPage) lv_obj_clear_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_add_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
    }

    // PRE_TRACK: show datetime label for clock display
    if (lbl_datetime) {
        if (isPreTrack) lv_obj_clear_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
    }

    lvglUnlock();

    // Reset cache to force immediate redraw on page entry
    memset(s_gpsStatusCache, 0, sizeof(s_gpsStatusCache));
    memset(&cache, 0, sizeof(cache));
    cache.lastBarValue = -9999;
    cache.lastLowBat = -1;
}

void setGpsStatusLine(int line, const char* text)
{
    if (line < 0 || line >= GPS_STATUS_LINE_COUNT || !gps_status_labels[line]) return;
    if (lvglLock(10)) {
        lv_label_set_text(gps_status_labels[line], text ? text : "");
        lvglUnlock();
    }
}

void updateGpsStatusDisplay(void)
{
    updateGpsStatusPage();
}

void updatePreTrackDisplay(const char* trackName)
{
    // Temporarily set pretrack state and call updateLapData which handles this path
    s_preTrackMode = true;
    if (trackName) {
        strncpy(s_preTrackName, trackName, sizeof(s_preTrackName) - 1);
        s_preTrackName[sizeof(s_preTrackName) - 1] = '\0';
    } else {
        s_preTrackName[0] = '\0';
    }
    updateLapData();
}

// ── 랩 서머리 표시 ──
// lapRowOffset  : 상단에 표시할 첫 번째 랩 인덱스 (sorted rank 기준)
// sectorColOffset: 섹터 열 스크롤 오프셋 (0 = 첫 번째 섹터부터)
void updateLapSummaryDisplay(int lapRowOffset, int sectorColOffset)
{
    if (!s_summaryOverlay) return;
    if (!lvglLock(50)) return;

    const int lapCount = gApp.sessionLapCount;
    if (lapCount == 0) {
        // 데이터 없음 — 간단한 안내 메시지
        lv_label_set_text(s_summaryCells[0][0], "NO LAPS");
        for (int r = 0; r < SUMMARY_ROWS; r++) {
            for (int c = 0; c < SUMMARY_TOTAL_COLS; c++) {
                if (r != 0 || c != 0) lv_label_set_text(s_summaryCells[r][c], "");
            }
        }
        lvglUnlock();
        return;
    }

    // 베스트 랩 기준 정렬 (인덱스 배열, 최대 30개)
    static int sortIdx[AppContext::MAX_SESSION_LAPS];
    int n = lapCount;
    for (int i = 0; i < n; i++) sortIdx[i] = i;
    // 삽입 정렬 (랩 수가 적으므로 충분)
    for (int i = 1; i < n; i++) {
        int key = sortIdx[i];
        uint32_t keyTime = gApp.sessionLaps[key].lapTimeMs;
        int j = i - 1;
        while (j >= 0 && gApp.sessionLaps[sortIdx[j]].lapTimeMs > keyTime) {
            sortIdx[j + 1] = sortIdx[j];
            j--;
        }
        sortIdx[j + 1] = key;
    }

    // 섹터 수 확인 (유효한 첫 번째 랩 기준)
    int sectorCount = 0;
    for (int i = 0; i < n; i++) {
        if (gApp.sessionLaps[i].sectorCount > sectorCount) {
            sectorCount = gApp.sessionLaps[i].sectorCount;
        }
    }

    char buf[32];

    // ── 헤더 행 (row 0) ──
    lv_label_set_text(s_summaryCells[0][0], "LAP");
    lv_label_set_text(s_summaryCells[0][1], "TIME");
    for (int sc = 0; sc < SUMMARY_SEC_COLS; sc++) {
        int sectorIdx = sc + sectorColOffset;
        if (sectorIdx < sectorCount) {
            snprintf(buf, sizeof(buf), "S%d", sectorIdx + 1);
            lv_label_set_text(s_summaryCells[0][2 + sc], buf);
        } else {
            lv_label_set_text(s_summaryCells[0][2 + sc], "");
        }
    }
    lv_label_set_text(s_summaryCells[0][8], "MIN");
    lv_label_set_text(s_summaryCells[0][9], "MAX");

    // ── 데이터 행 ──
    for (int row = 1; row < SUMMARY_ROWS; row++) {
        int rank = (row - 1) + lapRowOffset;
        if (rank >= n) {
            // 빈 행 클리어
            for (int c = 0; c < SUMMARY_TOTAL_COLS; c++) {
                lv_label_set_text(s_summaryCells[row][c], "");
            }
            continue;
        }

        const LapSummaryEntry& lap = gApp.sessionLaps[sortIdx[rank]];

        // 최고 랩(rank 0)은 색상 강조
        lv_color_t rowColor = (rank == 0) ? lv_color_make(80, 255, 80) : lv_color_white();
        for (int c = 0; c < SUMMARY_TOTAL_COLS; c++) {
            lv_obj_set_style_text_color(s_summaryCells[row][c], rowColor, 0);
        }

        // LAP 번호
        snprintf(buf, sizeof(buf), "L%02d", lap.lapNumber);
        lv_label_set_text(s_summaryCells[row][0], buf);

        // 랩 타임 (M:SS.s 형식)
        uint32_t ms = lap.lapTimeMs;
        uint32_t min = ms / 60000;
        uint32_t sec = (ms % 60000) / 1000;
        uint32_t dec = (ms % 1000) / 100;
        snprintf(buf, sizeof(buf), "%lu:%02lu.%lu", (unsigned long)min, (unsigned long)sec, (unsigned long)dec);
        lv_label_set_text(s_summaryCells[row][1], buf);

        // 섹터 타임 (SS.s 형식, 스크롤 적용)
        for (int sc = 0; sc < SUMMARY_SEC_COLS; sc++) {
            int sectorIdx = sc + sectorColOffset;
            if (sectorIdx < sectorCount && sectorIdx < lap.sectorCount
                    && lap.sectorTimesMs[sectorIdx] > 0) {
                uint32_t sms = lap.sectorTimesMs[sectorIdx];
                uint32_t ss = sms / 1000;
                uint32_t sd = (sms % 1000) / 100;
                snprintf(buf, sizeof(buf), "%lu.%lu", (unsigned long)ss, (unsigned long)sd);
                lv_label_set_text(s_summaryCells[row][2 + sc], buf);
            } else {
                lv_label_set_text(s_summaryCells[row][2 + sc], "");
            }
        }

        // MIN / MAX 속도 (정수 km/h)
        snprintf(buf, sizeof(buf), "%3d", (int)lap.minSpeedKmh);
        lv_label_set_text(s_summaryCells[row][8], buf);
        snprintf(buf, sizeof(buf), "%3d", (int)lap.maxSpeedKmh);
        lv_label_set_text(s_summaryCells[row][9], buf);
    }

    lvglUnlock();
}

void updateLaptimerDisplay(int userPageIndex)
{
    s_preTrackMode = false;
    s_userPageIndex = userPageIndex;
    updateLapData();
}

void updateLaptimerTimeDisplay(void)
{
    updateTimeData();
}

void updateLaptimerGpsDisplay(void)
{
    updateGpsData();
}

// ============================================================
// IMU STATUS PAGE
// ============================================================

void updateImuDisplay(void)
{
    if (!s_imuOverlay) return;

    // Read calibrated IMU accel (g units)
    float ax = gApp.imuData.accelX;
    float ay = gApp.imuData.accelY;
    float az = gApp.imuData.accelZ;

    // Remove gravity
    float gx = gApp.imuCalibration.gravityX;
    float gy = gApp.imuCalibration.gravityY;
    float gz = gApp.imuCalibration.gravityZ;
    float dx = ax - gx;
    float dy = ay - gy;
    float dz = az - gz;

    float fwdG = 0, latG = 0, vertG = 0;
    bool calibrated = axisCalibIsValid();

    // Get GPS heading for forward/lateral projection
    UBloxData ubx = getUBloxData();
    bool hasHeading = (ubx.valid && ubx.speedKmh > 5.0f);

    if (calibrated) {
        const float* R = axisCalibGetR();
        float aN = R[0] * dx + R[1] * dy + R[2] * dz;  // North (g)
        float aE = R[3] * dx + R[4] * dy + R[5] * dz;  // East (g)
        float aD = R[6] * dx + R[7] * dy + R[8] * dz;  // Down (g)

        if (hasHeading) {
            float h = ubx.headingDeg * (3.14159265f / 180.0f);
            fwdG =  aN * cosf(h) + aE * sinf(h);
            latG = -aN * sinf(h) + aE * cosf(h);
        } else {
            fwdG = aN;  // North as forward when no heading
            latG = aE;  // East as lateral
        }
        vertG = aD;
    } else {
        // Uncalibrated: raw sensor axes (arbitrary orientation)
        fwdG = dy;
        latG = dx;
        vertG = dz;
    }

    // G-force dot position (center = 0G, scale = GF_SCALE px/G)
    int dotCX = GF_SIZE / 2 + (int)(latG * GF_SCALE);
    int dotCY = GF_SIZE / 2 - (int)(fwdG * GF_SCALE);
    if (dotCX < 4) dotCX = 4;
    if (dotCX > GF_SIZE - 4) dotCX = GF_SIZE - 4;
    if (dotCY < 4) dotCY = 4;
    if (dotCY > GF_SIZE - 4) dotCY = GF_SIZE - 4;

    // Z-axis dot position (center = 0G, ±0.5G full range)
    int zDotY = ZB_HEIGHT / 2 - (int)(vertG * (ZB_HEIGHT / 1.0f));
    if (zDotY < 0) zDotY = 0;
    if (zDotY > ZB_HEIGHT - 8) zDotY = ZB_HEIGHT - 8;

    // Prepare info text (avoid snprintf inside mutex)
    char line1[40], line2[40], line3[40], line4[40];

    if (calibrated) {
        snprintf(line1, sizeof(line1), "Cal:OK  N=%d", axisCalibGetSampleCount());
        snprintf(line2, sizeof(line2), "RMS: %.2f m/s2", axisCalibGetResidual());
    } else {
        snprintf(line1, sizeof(line1), "Cal:--  N=%d", axisCalibGetSampleCount());
        snprintf(line2, sizeof(line2), "RMS: --");
    }
    snprintf(line3, sizeof(line3), "G:%.2f %.2f %.2f", gx, gy, gz);
    snprintf(line4, sizeof(line4), "F:%+.2f L:%+.2f Z:%+.3f", fwdG, latG, vertG);

    // Throttle: labels update at ~10Hz (every 6th call), dots at full rate
    static int s_imuFrameCount = 0;
    s_imuFrameCount++;
    bool slowUpdate = (s_imuFrameCount % 6 == 0);

    // LVGL updates
    if (!lvglLock(10)) return;

    // Dot positions (every frame — smooth movement)
    lv_obj_set_pos(s_gfDot, dotCX - 4, dotCY - 4);
    lv_obj_set_pos(s_zDot, 3, zDotY);

    if (slowUpdate) {
        lv_label_set_text(s_imuLabels[0], "IMU STATUS");
        lv_label_set_text(s_imuLabels[1], line1);
        lv_label_set_text(s_imuLabels[2], line2);
        lv_label_set_text(s_imuLabels[3], line3);
        lv_label_set_text(s_imuLabels[4], line4);
    }

    lvglUnlock();
}

