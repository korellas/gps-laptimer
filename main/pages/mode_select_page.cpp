/**
 * @file mode_select_page.cpp
 * @brief Startup mode selection page.
 *
 * Cycles through 7 modes via swipe left/right:
 *   0=LAPTIMER, 1=EMULATION, 2=GPS STATUS, 3=BLE OTA, 4=SETTINGS, 5=SD TEST, 6=IMU STATUS
 * Tap to confirm. Swipe up for phone plate.
 * Auto-enter LAPTIMER if speed > 50 km/h.
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "pages/transition_page.h"
#include "config.h"
#include "types.h"
#include "ublox_gps.h"

#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "ModeSelectPage";

struct ModeSelectPage : Page {
    int modeIndex = 0;
    GPSMode selectedMode = GPSMode::GPS_HARDWARE;
    bool initialized = false;

    ModeSelectPage()
        : Page(PageId::MODE_SELECT,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/true, SleepPolicy::IDLE_SLEEP)
    {}

    void onEnter(Page* from) override {
        initialized = false;
    }

    void onTick() override {
        if (modeIndex == 0 && updateUBloxGPS()) {
            UBloxData ubx = getUBloxData();
            if (ubx.valid && ubx.speedKmh > AUTO_ENTER_LAPTIMER_SPEED_KMH) {
                ESP_LOGI(TAG, "Auto-enter LAPTIMER (%.1f km/h)", ubx.speedKmh);
                gApp.currentGpsMode = GPSMode::GPS_HARDWARE;
                setTransitionTarget(PageId::PRE_TRACK, STARTUP_TRANSITION_MS);
                gPageManager.navigateTo(PageId::TRANSITION);
            }
        }
    }

    void onUpdate() override {
        if (!initialized) {
            modeIndex = (gApp.currentGpsMode == GPSMode::GPS_HARDWARE) ? 0 : 1;
            selectedMode = gApp.currentGpsMode;
            updateModeLabel();
            if (lvglLock(10)) {
                lv_label_set_text(getStartupHintLabel(), "swipe: change  /  tap: start");
                lv_obj_clear_flag(getStartupHintLabel(), LV_OBJ_FLAG_HIDDEN);
                lvglUnlock();
            }
            initialized = true;
        }
    }

    void onGesture(Gesture g) override {
        if (g == Gesture::SWIPE_LEFT || g == Gesture::SWIPE_RIGHT) {
            if (g == Gesture::SWIPE_LEFT) {
                modeIndex = (modeIndex + 1) % 7;
            } else {
                modeIndex = (modeIndex + 6) % 7;
            }
            if (modeIndex == 1) {
                selectedMode = GPSMode::SIMULATION;
            } else if (modeIndex != 3 && modeIndex != 4 && modeIndex != 5 && modeIndex != 6) {
                selectedMode = GPSMode::GPS_HARDWARE;
            }
            updateModeLabel();

        } else if (g == Gesture::SWIPE_UP) {
            gPageManager.navigateTo(PageId::PHONE_PLATE);

        } else if (g == Gesture::TAP) {
            confirmMode();
        }
    }

private:
    void updateModeLabel() {
        struct ModeInfo { const char* name; uint8_t r, g, b; };
        static const ModeInfo modes[] = {
            {"< LAPTIMER >",   0x20, 0xAA, 0x20},
            {"< EMULATION >",  0x00, 0xCC, 0xCC},
            {"< GPS STATUS >", 0xFF, 0xCC, 0x00},
            {"< BLE OTA >",    0x00, 0x88, 0xFF},
            {"< SETTINGS >",   0xFF, 0x88, 0x00},
            {"< SD TEST >",    0xFF, 0x44, 0x44},
            {"< IMU STATUS >", 0x88, 0xFF, 0x88},
        };
        const auto& m = modes[modeIndex];
        if (lvglLock(10)) {
            lv_obj_set_style_text_color(getStartupStatusLabel(),
                lv_color_make(m.r, m.g, m.b), 0);
            lv_label_set_text(getStartupStatusLabel(), m.name);
            lvglUnlock();
        }
    }

    void confirmMode() {
        const char* modeNames[] = {"LAPTIMER", "EMULATION", "GPS STATUS", "BLE OTA", "SETTINGS", "SD TEST", "IMU STATUS"};
        ESP_LOGI(TAG, "Mode selected: %s", modeNames[modeIndex]);

        if (modeIndex != 3 && modeIndex != 4 && modeIndex != 5 && modeIndex != 6) {
            gApp.currentGpsMode = selectedMode;
        }

        if (lvglLock(10)) {
            lv_obj_add_flag(getStartupHintLabel(), LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(getStartupVersionLabel(), APP_VERSION);
            if (modeIndex == 1) {
                lv_label_set_text(getStartupStatusLabel(), "EMULATION");
            } else if (modeIndex == 3) {
                lv_label_set_text(getStartupStatusLabel(), "BLE OTA");
            } else if (modeIndex == 4) {
                lv_label_set_text(getStartupStatusLabel(), "SETTINGS");
            } else if (modeIndex == 5) {
                lv_label_set_text(getStartupStatusLabel(), "SD TEST");
            } else if (modeIndex == 6) {
                lv_label_set_text(getStartupStatusLabel(), "IMU STATUS");
            }
            lvglUnlock();
        }

        switch (modeIndex) {
            case 0:  // LAPTIMER
                gPageManager.navigateTo(PageId::WAIT_GPS);
                break;
            case 1:  // EMULATION
                setTransitionTarget(PageId::EMULATION, STARTUP_SIM_DISPLAY_MS);
                gPageManager.navigateTo(PageId::TRANSITION);
                break;
            case 2:  // GPS STATUS
                setTransitionTarget(PageId::GPS_STATUS, STARTUP_TRANSITION_MS);
                gPageManager.navigateTo(PageId::TRANSITION);
                break;
            case 3:  // BLE OTA
                gPageManager.navigateTo(PageId::BLE_OTA);
                break;
            case 4:  // SETTINGS
                gPageManager.navigateTo(PageId::SETTINGS);
                break;
            case 5:  // SD TEST
                gPageManager.navigateTo(PageId::STORAGE_TEST);
                break;
            case 6:  // IMU STATUS
                gPageManager.navigateTo(PageId::IMU_STATUS);
                break;
        }
    }
};

static ModeSelectPage s_page;

Page* createModeSelectPage() { return &s_page; }
