/**
 * @file settings_page.cpp
 * @brief WiFi AP settings/OTA portal page.
 *
 * WiFi is started via manageSubsystems (needsWiFi=true).
 * Shows SSID and IP. Swipe to go back (unless OTA in progress).
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "config.h"
#include "types.h"
#include "wifi_portal.h"

#include <cstdio>

struct SettingsPage : Page {
    bool infoShown = false;

    SettingsPage()
        : Page(PageId::SETTINGS,
               /*gps=*/false, /*wifi=*/true, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        infoShown = false;
    }

    void onUpdate() override {
        if (!infoShown) {
            char infoBuf[96];
            snprintf(infoBuf, sizeof(infoBuf), "WiFi: " WIFI_AP_SSID "  /  " WIFI_AP_IP_STR);
            if (lvglLock(10)) {
                lv_obj_set_style_text_color(getStartupStatusLabel(),
                    lv_color_make(0xFF, 0x88, 0x00), 0);
                lv_label_set_text(getStartupStatusLabel(), "SETTINGS");
                lv_label_set_text(getStartupHintLabel(), infoBuf);
                lv_obj_clear_flag(getStartupHintLabel(), LV_OBJ_FLAG_HIDDEN);
                lvglUnlock();
            }
            infoShown = true;
        }

        // Update OTA progress if active
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
        }
    }

    void onGesture(Gesture g) override {
        if (g == Gesture::NONE) return;
        // Block exit during OTA
        bool exitBlocked = (gApp.otaState == OTAState::RECEIVING
                         || gApp.otaState == OTAState::VALIDATING
                         || gApp.otaState == OTAState::COMPLETE);
        if (!exitBlocked) {
            // WiFi stop is handled by manageSubsystems
            if (lvglLock(10)) {
                lv_obj_clear_flag(getStartupOverlay(), LV_OBJ_FLAG_HIDDEN);
                lvglUnlock();
            }
            gPageManager.navigateTo(PageId::MODE_SELECT);
        }
    }
};

static SettingsPage s_page;

Page* createSettingsPage() { return &s_page; }
