/**
 * @file transition_page.cpp
 * @brief Auto-advancing delay page (replaces TRANSITION + WAIT_SIM states).
 */

#include "pages/transition_page.h"
#include "page_manager.h"
#include "config.h"

#include "esp_timer.h"

static PageId        s_targetPageId = PageId::LAPTIMER;
static unsigned long s_delayMs      = STARTUP_TRANSITION_MS;
static unsigned long s_startMs      = 0;

struct TransitionPage : Page {
    TransitionPage()
        : Page(PageId::TRANSITION,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        s_startMs = (unsigned long)(esp_timer_get_time() / 1000);
    }

    void onTick() override {
        unsigned long now = (unsigned long)(esp_timer_get_time() / 1000);
        if ((now - s_startMs) >= s_delayMs) {
            gPageManager.navigateTo(s_targetPageId);
        }
    }
};

static TransitionPage s_page;

void setTransitionTarget(PageId target, unsigned long delayMs) {
    s_targetPageId = target;
    s_delayMs = delayMs;
}

Page* createTransitionPage() { return &s_page; }
