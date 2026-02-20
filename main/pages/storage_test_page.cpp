/**
 * @file storage_test_page.cpp
 * @brief SD card GPS logging diagnostic page.
 *
 * Opens a debug log via sdLoggerInit(), then polls the GPS in onTick() and
 * writes every incoming NAV-PVT frame to SD via sdLogGPS() — exactly the
 * same call path used during a real lap-timer session.
 *
 * Display (GPS-status label lines):
 *   0 — sdLoggerInit() result
 *   1 — GPS fix type / satellites / valid flag
 *   2 — Latitude / Longitude
 *   3 — Speed / Heading
 *   4 — SD point counter (live)
 *
 * Access: MODE_SELECT → swipe to "< SD TEST >" → tap
 * Exit:   TAP or SWIPE DOWN → sdLoggerClose() → MODE_SELECT
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "ublox_gps.h"
#include "sd_logger.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <cstdio>

static const char* TAG = "SDTEST";

struct StorageTestPage : Page {

    bool logStarted  = false;
    int  gpsReceived = 0;   // GPS frames received from module
    int  gpsWritten  = 0;   // GPS frames actually written to SD

    StorageTestPage()
        : Page(PageId::STORAGE_TEST,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        applyPageVisibilityForPage(PageId::STORAGE_TEST);
        gpsReceived = 0;
        gpsWritten  = 0;
        logStarted  = sdLoggerInit();

        ESP_LOGI(TAG, "sdLoggerInit() → %s", logStarted ? "OK" : "FAIL");

        if (logStarted) {
            setGpsStatusLine(0, "SD LOG: OK  recording GPS...");
        } else {
            setGpsStatusLine(0, "SD LOG: FAIL  (check SD card)");
        }
        setGpsStatusLine(1, "GPS: waiting...");
        setGpsStatusLine(2, "");
        setGpsStatusLine(3, "");
        setGpsStatusLine(4, "rx:0 wr:0  |  TAP: stop+exit");
    }

    void onExit(Page* to) override {
        sdLoggerClose();
        logStarted = false;
        ESP_LOGI(TAG, "Exiting SD test — recv=%d  written=%d", gpsReceived, gpsWritten);
    }

    void onTick() override {
        if (updateUBloxGPS()) {
            gpsReceived++;
            if (logStarted) {
                UBloxData ubx = getUBloxData();
                unsigned long ms = (unsigned long)(esp_timer_get_time() / 1000ULL);
                sdLogGPS(ms,
                         ubx.lat, ubx.lon,
                         ubx.speedKmh, ubx.headingDeg,
                         ubx.fixType, ubx.satellites,
                         "SD_TEST", 0);
                gpsWritten++;
            }
        }
    }

    void onUpdate() override {
        UBloxData ubx = getUBloxData();
        char buf[64];

        // Line 1: fix + sats
        static const char* const fixNames[] = {"NO FIX", "DR", "2D", "3D", "GNSS+DR", "?"};
        int fi = (ubx.fixType < 5) ? (int)ubx.fixType : 5;
        snprintf(buf, sizeof(buf), "GPS: %s  %d sat  %s",
                 fixNames[fi], ubx.satellites, ubx.valid ? "OK" : "wait");
        setGpsStatusLine(1, buf);

        // Line 2: position
        snprintf(buf, sizeof(buf), "%.6f  %.6f", ubx.lat, ubx.lon);
        setGpsStatusLine(2, buf);

        // Line 3: speed + heading
        snprintf(buf, sizeof(buf), "%.1f km/h  %.0f deg",
                 ubx.speedKmh, ubx.headingDeg);
        setGpsStatusLine(3, buf);

        // Line 4: counts + exit hint (always visible)
        snprintf(buf, sizeof(buf), "rx:%d wr:%d  |  TAP: stop+exit", gpsReceived, gpsWritten);
        setGpsStatusLine(4, buf);
    }

    void onGesture(Gesture g) override {
        if (g == Gesture::TAP || g == Gesture::SWIPE_DOWN) {
            gPageManager.navigateTo(PageId::MODE_SELECT);
        }
    }
};

static StorageTestPage s_page;

Page* createStorageTestPage() { return &s_page; }
