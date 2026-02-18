/**
 * @file pre_track_page.cpp
 * @brief Pre-track page — speed + clock display before session starts.
 *
 * Active during PRE_TRACK GPS session state.
 * processRealGPS() checks all finish lines for track identification and session start.
 * When session starts, PreTrackPage detects SESSION_ACTIVE and navigates to LaptimerPage.
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "gps_processor.h"
#include "track_manager.h"

struct PreTrackPage : Page {
    bool sessionStarted = false;

    PreTrackPage()
        : Page(PageId::PRE_TRACK,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::FULL,
               /*sleep=*/true, SleepPolicy::IDLE_SLEEP)
    {}

    void onEnter(Page* from) override {
        sessionStarted = false;
        applyPageVisibilityForPage(PageId::PRE_TRACK);
    }

    void onTick() override {
        // Detect session start: gps_processor transitions to SESSION_ACTIVE
        if (!sessionStarted && getGPSSessionState() == GPSSessionState::SESSION_ACTIVE) {
            sessionStarted = true;
            gPageManager.navigateTo(PageId::LAPTIMER);
        }
    }

    void onUpdate() override {
        // Track name from active track (set when finish line crossed)
        const ActiveTrack& active = getActiveTrackConst();
        updatePreTrackDisplay(active.isValid() ? active.track->name : nullptr);
    }
};

static PreTrackPage s_page;

Page* createPreTrackPage() { return &s_page; }
