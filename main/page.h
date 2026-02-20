/**
 * @file page.h
 * @brief Page abstraction base struct and enums for the app-level page system.
 *
 * Each screen/mode of the app is a Page subclass with its own lifecycle,
 * gesture handling, and declared subsystem requirements.
 */

#pragma once

#include <cstdint>

/**
 * Identity of every app-level page.
 * Used by PageManager for navigation and by external modules (via callback)
 * to request page transitions without depending on the display layer.
 */
enum class PageId : uint8_t {
    MODE_SELECT,
    PHONE_PLATE,
    WAIT_GPS,
    BLE_OTA,
    SETTINGS,
    GPS_STATUS,
    IMU_STATUS,
    PRE_TRACK,
    LAPTIMER,
    EMULATION,
    TRANSITION,
    LAP_SUMMARY,
    STORAGE_TEST,
    PAGE_COUNT
};

/**
 * Sleep policy per page (consumed by future sleep manager).
 */
enum class SleepPolicy : uint8_t {
    NO_SLEEP,       // never auto-sleep on this page
    IDLE_SLEEP,     // auto-sleep after idle timeout
};

/**
 * How main_task drives session processing for the active page.
 */
enum class ProcessingMode : uint8_t {
    NONE,   // no session processing (startup pages, GPS status, etc.)
    FULL,   // processRealGPS() or processSimulation()
};

/**
 * Gesture results passed to Page::onGesture().
 * Mirrors the GestureResult values in waveshare_display.cpp.
 */
enum class Gesture : uint8_t {
    NONE = 0,
    TAP,
    SWIPE_LEFT,
    SWIPE_RIGHT,
    SWIPE_UP,
    SWIPE_DOWN,
};

/**
 * Base struct for all app pages.
 *
 * Each page is a static global — no heap allocation.
 * Virtual methods keep the code clean; vtable overhead is ~40 bytes
 * total across all 10 page instances (negligible on ESP32-S3).
 */
struct Page {
    // --- Immutable properties (set at construction) ---
    const PageId         id;
    const bool           needsGPS;          // GPS module must be ON
    const bool           needsWiFi;         // WiFi AP must be running
    const bool           needsBLE;          // BLE stack must be active
    const ProcessingMode processingMode;    // how main_task drives this page
    const bool           allowsSleep;       // for future sleep feature
    const SleepPolicy    sleepPolicy;

    // --- Lifecycle (override per page) ---

    /// Called when navigating TO this page. |from| is the previous page (may be null).
    virtual void onEnter(Page* from)       {}

    /// Called when navigating AWAY from this page. |to| is the next page.
    virtual void onExit(Page* to)          {}

    /// Called every main_task tick (~5ms). For fast polling (GPS UART etc.).
    virtual void onTick()                  {}

    /// Called at display refresh rate (~60Hz). For LVGL updates.
    virtual void onUpdate()                {}

    /// Called when a gesture is detected on this page.
    virtual void onGesture(Gesture g)      {}

    virtual ~Page() = default;

protected:
    Page(PageId id_, bool gps, bool wifi, bool ble,
         ProcessingMode pm, bool sleep, SleepPolicy sp)
        : id(id_)
        , needsGPS(gps)
        , needsWiFi(wifi)
        , needsBLE(ble)
        , processingMode(pm)
        , allowsSleep(sleep)
        , sleepPolicy(sp)
    {}
};
