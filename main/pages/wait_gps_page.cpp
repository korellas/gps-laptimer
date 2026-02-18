/**
 * @file wait_gps_page.cpp
 * @brief GPS 3D fix wait screen with satellite status animation.
 *
 * Polls GPS in onTick(), updates display in onUpdate().
 * After 3D fix held for 1 second, transitions to target page.
 * After 60s timeout, shows "No GPS - tap to skip".
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "pages/transition_page.h"
#include "config.h"
#include "types.h"
#include "ublox_gps.h"
#include "waveshare_display.h"

#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

struct WaitGpsPage : Page {
    unsigned long createdMs = 0;
    unsigned long fixAcquiredMs = 0;
    char statusCache[64] = "";

    WaitGpsPage()
        : Page(PageId::WAIT_GPS,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        createdMs = (unsigned long)(esp_timer_get_time() / 1000);
        fixAcquiredMs = 0;
        statusCache[0] = '\0';
    }

    void onTick() override {
        updateUBloxGPS();

        UBloxData ubx = getUBloxData();
        unsigned long now = (unsigned long)(esp_timer_get_time() / 1000);

        if (ubx.fixType >= 3) {
            // 3D fix acquired
            if (fixAcquiredMs == 0) fixAcquiredMs = now;
            if ((now - fixAcquiredMs) >= 1000) {
                // Fix held for 1 second — transition to PRE_TRACK
                setTransitionTarget(PageId::PRE_TRACK, STARTUP_TRANSITION_MS);
                gPageManager.navigateTo(PageId::TRANSITION);
            }
        } else {
            fixAcquiredMs = 0;
        }

        // Timeout: tap to skip (go to PRE_TRACK even without fix)
        if ((now - createdMs) >= STARTUP_GPS_TIMEOUT_MS) {
            if (readTouch()) {
                setTransitionTarget(PageId::PRE_TRACK, STARTUP_TRANSITION_MS);
                gPageManager.navigateTo(PageId::TRANSITION);
            }
        }
    }

    void onUpdate() override {
        UBloxData ubx = getUBloxData();
        unsigned long now = (unsigned long)(esp_timer_get_time() / 1000);
        char statusBuf[64];

        if (ubx.fixType >= 3) {
            snprintf(statusBuf, sizeof(statusBuf), "3D Fix (%d sats)", ubx.satellites);
            if (strcmp(statusBuf, statusCache) != 0) {
                if (lvglLock(10)) {
                    lv_obj_set_style_text_color(getStartupStatusLabel(),
                        lv_color_make(0x20, 0xAA, 0x20), 0);
                    lv_label_set_text(getStartupStatusLabel(), statusBuf);
                    lvglUnlock();
                }
                strcpy(statusCache, statusBuf);
            }
        } else if (ubx.fixType == 2) {
            snprintf(statusBuf, sizeof(statusBuf), "2D Fix (%d sats)", ubx.satellites);
            if (strcmp(statusBuf, statusCache) != 0) {
                if (lvglLock(10)) {
                    lv_obj_set_style_text_color(getStartupStatusLabel(),
                        lv_color_make(0xFF, 0xCC, 0x00), 0);
                    lv_label_set_text(getStartupStatusLabel(), statusBuf);
                    lvglUnlock();
                }
                strcpy(statusCache, statusBuf);
            }
        } else {
            bool timedOut = (now - createdMs) >= STARTUP_GPS_TIMEOUT_MS;

            if (!timedOut) {
                const char* spinner[] = {"-  ", " - ", "  -", " - "};
                int phase = (int)((now / 300) % 4);
                snprintf(statusBuf, sizeof(statusBuf), "[%s] %d sats",
                         spinner[phase], ubx.satellites);
                if (strcmp(statusBuf, statusCache) != 0) {
                    if (lvglLock(10)) {
                        lv_obj_set_style_text_color(getStartupStatusLabel(),
                            lv_color_make(0xFF, 0xCC, 0x00), 0);
                        lv_label_set_text(getStartupStatusLabel(), statusBuf);
                        lvglUnlock();
                    }
                    strcpy(statusCache, statusBuf);
                }
            } else {
                const char *noGpsMsg = "No GPS - tap to skip";
                if (strcmp(noGpsMsg, statusCache) != 0) {
                    if (lvglLock(10)) {
                        lv_obj_set_style_text_color(getStartupStatusLabel(),
                            lv_color_make(0xFF, 0x60, 0x00), 0);
                        lv_label_set_text(getStartupStatusLabel(), noGpsMsg);
                        lvglUnlock();
                    }
                    strcpy(statusCache, noGpsMsg);
                }
            }
        }
    }
};

static WaitGpsPage s_page;

Page* createWaitGpsPage() { return &s_page; }
