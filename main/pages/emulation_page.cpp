/**
 * @file emulation_page.cpp
 * @brief Simulation mode — same display as LaptimerPage, GPS off.
 *
 * processSimulation() runs via processingMode=FULL.
 * Shares display functions with LaptimerPage.
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "display_config.h"
#include "waveshare_display.h"

struct EmulationPage : Page {
    int userPageIndex = 0;

    EmulationPage()
        : Page(PageId::EMULATION,
               /*gps=*/false, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::FULL,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        userPageIndex = 0;
        applyPageVisibilityForPage(PageId::LAPTIMER);  // same layout as laptimer
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

static EmulationPage s_page;

Page* createEmulationPage() { return &s_page; }
