/**
 * @file laptimer_page.cpp
 * @brief Main racing page — delta, lap time, sectors.
 *
 * Active during SESSION_ACTIVE GPS state.
 * Swipe left/right cycles through user sub-pages (display_config.json).
 * Shares display logic with EmulationPage via common functions.
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "display_config.h"
#include "waveshare_display.h"
#include "gps_processor.h"

#include <cstring>

struct LaptimerPage : Page {
    int userPageIndex = 0;

    LaptimerPage()
        : Page(PageId::LAPTIMER,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::FULL,
               /*sleep=*/true, SleepPolicy::IDLE_SLEEP)
    {}

    void onEnter(Page* from) override {
        userPageIndex = 0;
        resetDeltaHistory();
        applyPageVisibilityForPage(PageId::LAPTIMER);
    }

    void onTick() override {
        GPSSessionState st = getGPSSessionState();
        if (st == GPSSessionState::SESSION_ENDING) {
            gPageManager.navigateTo(PageId::LAP_SUMMARY);
        } else if (st != GPSSessionState::SESSION_ACTIVE) {
            gPageManager.navigateTo(PageId::PRE_TRACK);
        }
    }

    void onUpdate() override {
        if (tvalid) updateLaptimerTimeDisplay();
        if (lvalid) updateLaptimerDisplay(userPageIndex);
        if (gvalid) updateLaptimerGpsDisplay();
    }

    void onGesture(Gesture g) override {
        if (g == Gesture::SWIPE_LEFT || g == Gesture::SWIPE_RIGHT) {
            const int pageCount = getDisplayConfig().pageCount;
            if (pageCount > 1) {
                userPageIndex = (userPageIndex + 1) % pageCount;
            }
            applyPageVisibilityForPage(PageId::LAPTIMER);
        }
    }
};

static LaptimerPage s_page;

Page* createLaptimerPage() { return &s_page; }
