/**
 * @file ble_ota_page.cpp
 * @brief BLE OTA firmware update page.
 *
 * BLE is started via manageSubsystems (needsBLE=true).
 * Shows BLE advertising status and OTA progress.
 * Swipe to go back (unless OTA in progress).
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "types.h"
#include "ble_ota.h"

#include <cstdio>

struct BleOtaPage : Page {
    bool started = false;
    bool failed = false;

    BleOtaPage()
        : Page(PageId::BLE_OTA,
               /*gps=*/false, /*wifi=*/false, /*ble=*/true,
               ProcessingMode::NONE,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        started = false;
        failed = false;
        // BLE start is handled by manageSubsystems.
        // But we need to check if it actually started (might fail).
        if (isBleOtaActive()) {
            started = true;
            if (lvglLock(10)) {
                lv_label_set_text(getStartupStatusLabel(), "BLE OTA");
                lv_obj_set_style_text_color(getStartupStatusLabel(),
                    lv_color_make(0x00, 0x88, 0xFF), 0);
                lv_label_set_text(getStartupHintLabel(), "starting...  swipe: back");
                lv_obj_clear_flag(getStartupHintLabel(), LV_OBJ_FLAG_HIDDEN);
                lvglUnlock();
            }
        } else {
            failed = true;
            if (lvglLock(10)) {
                lv_label_set_text(getStartupStatusLabel(), "BLE OTA");
                lv_obj_set_style_text_color(getStartupStatusLabel(),
                    lv_color_make(0xFF, 0x00, 0x00), 0);
                lv_label_set_text(getStartupHintLabel(), "init failed!  swipe: back");
                lv_obj_clear_flag(getStartupHintLabel(), LV_OBJ_FLAG_HIDDEN);
                lvglUnlock();
            }
        }
    }

    void onUpdate() override {
        if (!started) return;

        if (gApp.otaState == OTAState::RECEIVING) {
            char buf[48];
            snprintf(buf, sizeof(buf), "updating... %d%%", (int)(gApp.otaProgress * 100));
            if (lvglLock(10)) {
                lv_label_set_text(getStartupHintLabel(), buf);
                lvglUnlock();
            }
        } else if (gApp.otaState == OTAState::COMPLETE) {
            if (lvglLock(10)) {
                lv_label_set_text(getStartupHintLabel(), "update complete! rebooting...");
                lvglUnlock();
            }
        } else if (gApp.otaState == OTAState::ERROR) {
            if (lvglLock(10)) {
                lv_label_set_text(getStartupHintLabel(), "update failed!  swipe: back");
                lvglUnlock();
            }
        } else if (isBleOtaAdvertising()) {
            if (lvglLock(10)) {
                lv_label_set_text(getStartupHintLabel(), "advertising...  swipe: back");
                lvglUnlock();
            }
        }
    }

    void onGesture(Gesture g) override {
        if (g == Gesture::NONE) return;
        bool exitBlocked = (gApp.otaState == OTAState::RECEIVING
                         || gApp.otaState == OTAState::VALIDATING
                         || gApp.otaState == OTAState::COMPLETE);
        if (!exitBlocked) {
            // BLE stop is handled by manageSubsystems
            if (lvglLock(10)) {
                lv_obj_clear_flag(getStartupOverlay(), LV_OBJ_FLAG_HIDDEN);
                lvglUnlock();
            }
            gPageManager.navigateTo(PageId::MODE_SELECT);
        }
    }
};

static BleOtaPage s_page;

Page* createBleOtaPage() { return &s_page; }
