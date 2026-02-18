/**
 * @file gps_status_page.cpp
 * @brief GPS diagnostic page — satellite info, fix status.
 *
 * Polls GPS in onTick() (self-contained, processingMode=NONE).
 * No gesture handling (entered from startup only).
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "ublox_gps.h"

struct GpsStatusPage : Page {
    GpsStatusPage()
        : Page(PageId::GPS_STATUS,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        enableNavSatOutput(true);
        applyPageVisibilityForPage(PageId::GPS_STATUS);
    }

    void onExit(Page* to) override {
        enableNavSatOutput(false);
    }

    void onTick() override {
        updateUBloxGPS();
    }

    void onUpdate() override {
        updateGpsStatusDisplay();
    }
};

static GpsStatusPage s_page;

Page* createGpsStatusPage() { return &s_page; }
