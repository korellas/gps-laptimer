/**
 * @file phone_plate_page.cpp
 * @brief Fullscreen phone number display page.
 *
 * Entered from ModeSelectPage via swipe up.
 * Any gesture returns to ModeSelectPage.
 * GPS is OFF on this page (just showing a phone number for parking).
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "types.h"

struct PhonePlatePage : Page {
    PhonePlatePage()
        : Page(PageId::PHONE_PLATE,
               /*gps=*/false, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/true, SleepPolicy::IDLE_SLEEP)
    {}

    void onEnter(Page* from) override {
        if (lvglLock(10)) {
            // Show phone number
            if (gApp.phoneNumber[0] != '\0') {
                lv_label_set_text(getPhoneNumberLabel(), gApp.phoneNumber);
            }
            lv_obj_add_flag(getStartupOverlay(), LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(getPhoneOverlay(), LV_OBJ_FLAG_HIDDEN);
            lvglUnlock();
        }
    }

    void onExit(Page* to) override {
        if (lvglLock(10)) {
            lv_obj_add_flag(getPhoneOverlay(), LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(getStartupOverlay(), LV_OBJ_FLAG_HIDDEN);
            lvglUnlock();
        }
    }

    void onGesture(Gesture g) override {
        if (g != Gesture::NONE) {
            gPageManager.goBack();  // return to ModeSelectPage
        }
    }
};

static PhonePlatePage s_page;

Page* createPhonePlatePage() { return &s_page; }
