/**
 * @file page_manager.h
 * @brief Manages page lifecycle, navigation, and subsystem transitions.
 */

#pragma once

#include "page.h"

class PageManager {
public:
    /// Register a page instance (called during init for each page).
    void registerPage(Page* page);

    /// Navigate to a page by ID. Calls onExit → subsystem management → onEnter.
    void navigateTo(PageId target);

    /// Return to the previous page (one level only).
    void goBack();

    /// Get the currently active page.
    Page* current() const { return m_current; }

    /// Shortcut for current()->id.
    PageId currentId() const { return m_current ? m_current->id : PageId::MODE_SELECT; }

    /// Get a page by ID (for setting page-specific state before navigation).
    Page* page(PageId id) const { return m_pages[static_cast<int>(id)]; }

    /// Called every main_task tick (~5ms).
    /// Runs session processing (if FULL mode) then the page's onTick().
    void tick();

    /// Called at display refresh rate (~60Hz).
    /// Detects gestures, routes to onGesture(), then calls onUpdate().
    void update();

private:
    /// Start/stop GPS, WiFi, BLE based on old vs new page requirements.
    void manageSubsystems(Page* oldPage, Page* newPage);

    /// Record user activity (touch/gesture detected). Wakes screen if off.
    void recordActivity();

    /// Check idle timeouts — screen off and auto power-off.
    void checkIdleTimeouts();

    /// Whether the current page + session state allows sleep.
    bool sleepAllowed() const;

    Page* m_pages[static_cast<int>(PageId::PAGE_COUNT)] = {};
    Page* m_current  = nullptr;
    Page* m_previous = nullptr;
    bool  m_modeInit = false;   // one-time init for FULL processing mode

    // Sleep / idle tracking
    uint32_t m_lastActivityMs = 0;
    bool     m_screenOff      = false;
};

extern PageManager gPageManager;

/// Initialize all pages and set initial page to MODE_SELECT.
/// Call once during app_init(), after display and GPS are ready.
void initPageSystem();
