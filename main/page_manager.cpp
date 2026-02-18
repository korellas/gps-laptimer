/**
 * @file page_manager.cpp
 * @brief Page lifecycle manager — navigation, subsystem transitions, gesture routing,
 *        idle tracking, screen sleep, and auto power-off.
 */

#include "page_manager.h"
#include "display_widgets.h"
#include "waveshare_display.h"
#include "config.h"

#include "ublox_gps.h"
#include "wifi_portal.h"
#include "ble_ota.h"
#include "gps_processor.h"
#include "simulation.h"
#include "types.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"

static const char* TAG = "PageMgr";

// Global instance
PageManager gPageManager;

static inline uint32_t pm_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ----------------------------------------------------------------
// Page factory functions (defined in pages/*.cpp)
// ----------------------------------------------------------------
extern Page* createModeSelectPage();
extern Page* createPhonePlatePage();
extern Page* createWaitGpsPage();
extern Page* createBleOtaPage();
extern Page* createSettingsPage();
extern Page* createGpsStatusPage();
extern Page* createPreTrackPage();
extern Page* createLaptimerPage();
extern Page* createEmulationPage();
extern Page* createTransitionPage();

void initPageSystem()
{
    gPageManager.registerPage(createModeSelectPage());
    gPageManager.registerPage(createPhonePlatePage());
    gPageManager.registerPage(createWaitGpsPage());
    gPageManager.registerPage(createBleOtaPage());
    gPageManager.registerPage(createSettingsPage());
    gPageManager.registerPage(createGpsStatusPage());
    gPageManager.registerPage(createPreTrackPage());
    gPageManager.registerPage(createLaptimerPage());
    gPageManager.registerPage(createEmulationPage());
    gPageManager.registerPage(createTransitionPage());

    // Start on MODE_SELECT
    gPageManager.navigateTo(PageId::MODE_SELECT);

    ESP_LOGI(TAG, "Page system initialized (%d pages)", static_cast<int>(PageId::PAGE_COUNT));
}

// ----------------------------------------------------------------
// Registration
// ----------------------------------------------------------------

void PageManager::registerPage(Page* page)
{
    int idx = static_cast<int>(page->id);
    if (idx >= 0 && idx < static_cast<int>(PageId::PAGE_COUNT)) {
        m_pages[idx] = page;
    }
}

// ----------------------------------------------------------------
// Navigation
// ----------------------------------------------------------------

void PageManager::navigateTo(PageId target)
{
    // Navigation counts as activity; wake screen if off
    m_lastActivityMs = pm_millis();
    if (m_screenOff) {
        setBacklight(true);
        m_screenOff = false;
    }

    Page* newPage = m_pages[static_cast<int>(target)];
    if (!newPage) {
        ESP_LOGE(TAG, "navigateTo: page %d not registered", static_cast<int>(target));
        return;
    }
    if (m_current == newPage) {
        return;  // already on this page
    }

    ESP_LOGI(TAG, "Navigate: %d -> %d",
             m_current ? static_cast<int>(m_current->id) : -1,
             static_cast<int>(target));

    // Exit current page
    if (m_current) {
        m_current->onExit(newPage);
    }

    // Manage hardware subsystems based on page requirements
    manageSubsystems(m_current, newPage);

    // Switch pages
    m_previous = m_current;
    m_current = newPage;

    // Reset mode init when switching to a FULL processing page
    if (m_current->processingMode == ProcessingMode::FULL &&
        (!m_previous || m_previous->processingMode != ProcessingMode::FULL)) {
        m_modeInit = false;
    }

    // Enter new page
    m_current->onEnter(m_previous);
}

void PageManager::goBack()
{
    if (m_previous) {
        navigateTo(m_previous->id);
    }
}

// ----------------------------------------------------------------
// Subsystem management
// ----------------------------------------------------------------

void PageManager::manageSubsystems(Page* oldPage, Page* newPage)
{
    bool oldGPS  = oldPage ? oldPage->needsGPS  : false;
    bool oldWiFi = oldPage ? oldPage->needsWiFi : false;
    bool oldBLE  = oldPage ? oldPage->needsBLE  : false;

    // GPS: enable/disable based on page requirement
    if (newPage->needsGPS && !oldGPS) {
        ESP_LOGI(TAG, "Subsystem: GPS ON");
        enableGPSModule();
    } else if (!newPage->needsGPS && oldGPS) {
        ESP_LOGI(TAG, "Subsystem: GPS OFF");
        disableGPSModule();
    }

    // WiFi: start/stop portal
    if (newPage->needsWiFi && !oldWiFi) {
        ESP_LOGI(TAG, "Subsystem: WiFi ON");
        startWifiPortal();
    } else if (!newPage->needsWiFi && oldWiFi) {
        ESP_LOGI(TAG, "Subsystem: WiFi OFF");
        stopWifiPortal();
    }

    // BLE: special handling — WiFi must be deinitialized first
    if (newPage->needsBLE && !oldBLE) {
        if (isWifiPortalActive()) {
            stopWifiPortal();
        }
        esp_wifi_deinit();
        ESP_LOGI(TAG, "Subsystem: BLE ON (WiFi deinitialized)");
        startBleOta();
    } else if (!newPage->needsBLE && oldBLE) {
        ESP_LOGI(TAG, "Subsystem: BLE OFF");
        stopBleOta();
    }
}

// ----------------------------------------------------------------
// Tick (called every ~5ms from main_task)
// ----------------------------------------------------------------

void PageManager::tick()
{
    if (!m_current) return;

    // Session processing for FULL mode pages
    if (m_current->processingMode == ProcessingMode::FULL) {
        if (!m_modeInit) {
            if (gApp.currentGpsMode == GPSMode::SIMULATION) {
                initializeSimulation();
            } else {
                initializeGPSMode();
            }
            m_modeInit = true;
        }

        if (gApp.currentGpsMode == GPSMode::SIMULATION) {
            processSimulation();
        } else {
            processRealGPS();
        }
    }

    // Page-specific fast polling (GPS UART, etc.)
    m_current->onTick();
}

// ----------------------------------------------------------------
// Update (called at ~60Hz display rate)
// ----------------------------------------------------------------

void PageManager::update()
{
    if (!m_current) return;

    // Cross-page concerns
    updateNotificationDisplay();
    updateBatteryWarning();

    // Check raw touch state for idle tracking
    bool touched = readTouch();

    // Route gestures to current page
    Gesture g = pollGesture();

    // Record activity on any touch, gesture, or vehicle movement
    if (touched || g != Gesture::NONE) {
        recordActivity();
    }
    if (m_current->needsGPS && gApp.currentPoint.speedKmh > MOVEMENT_WAKE_SPEED_KMH) {
        recordActivity();
    }

    // Screen is off: first touch wakes only (don't route gesture to page)
    if (m_screenOff) {
        checkIdleTimeouts();
        return;
    }

    if (g != Gesture::NONE) {
        m_current->onGesture(g);
    }

    // Page display update
    m_current->onUpdate();

    // Check idle timeouts
    checkIdleTimeouts();
}

// ----------------------------------------------------------------
// Sleep / idle tracking
// ----------------------------------------------------------------

void PageManager::recordActivity()
{
    m_lastActivityMs = pm_millis();

    if (m_screenOff) {
        setBacklight(true);
        m_screenOff = false;
        ESP_LOGI(TAG, "Wake: touch detected");
    }
}

bool PageManager::sleepAllowed() const
{
    if (!m_current) return false;
    if (m_current->sleepPolicy != SleepPolicy::IDLE_SLEEP) return false;

    // Never sleep during active racing session
    if (getGPSSessionState() == GPSSessionState::SESSION_ACTIVE) return false;

    return true;
}

void PageManager::checkIdleTimeouts()
{
    if (!sleepAllowed()) {
        // Wake screen if it was off (e.g., session just started)
        if (m_screenOff) {
            setBacklight(true);
            m_screenOff = false;
            ESP_LOGI(TAG, "Wake: sleep no longer allowed");
        }
        // Keep activity timestamp fresh while sleep is not allowed
        m_lastActivityMs = pm_millis();
        return;
    }

    uint32_t now = pm_millis();
    uint32_t idleMs = now - m_lastActivityMs;

    // Auto power-off (longer timeout)
    if (idleMs >= AUTO_POWEROFF_TIMEOUT_MS) {
        ESP_LOGI(TAG, "Auto power-off: idle %lu ms", (unsigned long)idleMs);
        systemPowerOff();
        return;
    }

    // Screen off (shorter timeout)
    if (!m_screenOff && idleMs >= SCREEN_OFF_TIMEOUT_MS) {
        setBacklight(false);
        m_screenOff = true;
        ESP_LOGI(TAG, "Screen off: idle %lu ms", (unsigned long)idleMs);
    }
}
