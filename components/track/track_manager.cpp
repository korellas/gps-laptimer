/**
 * @file track_manager.cpp
 * @brief Track detection and selection management implementation
 * @version 1.0
 */

#include "track_manager.h"
#include "config.h"

#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "TrackMgr";

// ============================================================
// STATIC STATE
// ============================================================

static ActiveTrack s_activeTrack;
static bool s_initialized = false;


// ============================================================
// INITIALIZATION
// ============================================================

void initTrackManager() {
    resetTrackManager();
    s_initialized = true;
    
    ESP_LOGI(TAG, "Initialized with %d built-in tracks", BUILTIN_TRACK_COUNT);
}

void resetTrackManager() {
    s_activeTrack.clear();
}

// ============================================================
// ACTIVE TRACK MANAGEMENT
// ============================================================

ActiveTrack& getActiveTrack() {
    return s_activeTrack;
}

const ActiveTrack& getActiveTrackConst() {
    return s_activeTrack;
}

bool hasActiveTrack() {
    return s_activeTrack.isValid();
}

bool setActiveTrack(const TrackDefinition* track, int layoutIndex) {
    if (track == nullptr) {
        clearActiveTrack();
        return false;
    }
    
    // Validate layout index
    if (layoutIndex < 0 || layoutIndex >= track->layoutCount) {
        layoutIndex = 0;  // Default to first layout
    }
    
    s_activeTrack.track = track;
    s_activeTrack.layout = track->getLayout(layoutIndex);
    s_activeTrack.layoutIndex = layoutIndex;
    s_activeTrack.userConfirmed = false;
    
    ESP_LOGI(TAG, "Active track: %s, layout: %s",
             track->name, s_activeTrack.layout->name);
    
    return true;
}

bool setActiveTrackById(const char* trackId, const char* layoutId) {
    const TrackDefinition* track = getBuiltinTrackById(trackId);
    if (track == nullptr) {
        ESP_LOGW(TAG, "Track not found: %s", trackId);
        return false;
    }
    
    int layoutIndex = 0;
    if (layoutId != nullptr) {
        // Find layout by ID
        for (int i = 0; i < track->layoutCount; i++) {
            if (strcmp(track->layouts[i].id, layoutId) == 0) {
                layoutIndex = i;
                break;
            }
        }
    }
    
    return setActiveTrack(track, layoutIndex);
}

void clearActiveTrack() {
    s_activeTrack.clear();
    ESP_LOGI(TAG, "Active track cleared");
}

void confirmActiveTrack() {
    if (s_activeTrack.isValid()) {
        s_activeTrack.userConfirmed = true;
        ESP_LOGI(TAG, "Track confirmed: %s", s_activeTrack.track->name);
    }
}

// ============================================================
// LAYOUT SELECTION
// ============================================================

bool setActiveLayout(int layoutIndex) {
    if (!s_activeTrack.isValid()) {
        return false;
    }
    
    if (layoutIndex < 0 || layoutIndex >= s_activeTrack.track->layoutCount) {
        return false;
    }
    
    s_activeTrack.layout = s_activeTrack.track->getLayout(layoutIndex);
    s_activeTrack.layoutIndex = layoutIndex;
    
    ESP_LOGI(TAG, "Layout changed: %s", s_activeTrack.layout->name);
    
    return true;
}

bool setActiveLayoutById(const char* layoutId) {
    if (!s_activeTrack.isValid() || layoutId == nullptr) {
        return false;
    }
    
    for (int i = 0; i < s_activeTrack.track->layoutCount; i++) {
        if (strcmp(s_activeTrack.track->layouts[i].id, layoutId) == 0) {
            return setActiveLayout(i);
        }
    }
    
    return false;
}

bool nextLayout() {
    if (!s_activeTrack.isValid()) {
        return false;
    }
    
    int newIndex = (s_activeTrack.layoutIndex + 1) % s_activeTrack.track->layoutCount;
    return setActiveLayout(newIndex);
}

bool prevLayout() {
    if (!s_activeTrack.isValid()) {
        return false;
    }
    
    int newIndex = s_activeTrack.layoutIndex - 1;
    if (newIndex < 0) {
        newIndex = s_activeTrack.track->layoutCount - 1;
    }
    return setActiveLayout(newIndex);
}

int getActiveLayoutCount() {
    if (!s_activeTrack.isValid()) {
        return 0;
    }
    return s_activeTrack.track->layoutCount;
}

int getActiveLayoutIndex() {
    if (!s_activeTrack.isValid()) {
        return -1;
    }
    return s_activeTrack.layoutIndex;
}

// ============================================================
// FINISH LINE ACCESS
// ============================================================

const FinishLineDefinition* getActiveFinishLineDefinition() {
    if (!s_activeTrack.isValid()) {
        return nullptr;
    }
    return &s_activeTrack.layout->finishLine;
}

bool hasActiveFinishLineDefinition() {
    const FinishLineDefinition* fl = getActiveFinishLineDefinition();
    return fl != nullptr && fl->isConfigured();
}

// ============================================================
// SECTOR ACCESS
// ============================================================

int getActiveSectorCount() {
    if (!s_activeTrack.isValid()) {
        return 0;
    }
    return s_activeTrack.layout->sectorCount;
}

const Sector* getActiveSector(int index) {
    if (!s_activeTrack.isValid()) {
        return nullptr;
    }
    return s_activeTrack.layout->getSector(index);
}

bool hasActiveSectors() {
    return getActiveSectorCount() > 0;
}

// ============================================================
// REFERENCE LAP ACCESS
// ============================================================

bool hasBuiltinReference() {
    if (!s_activeTrack.isValid()) {
        return false;
    }
    return s_activeTrack.layout->hasReference();
}

bool getBuiltinReferenceInfo(int& outStartIdx, int& outEndIdx, uint32_t& outTimeMs) {
    if (!hasBuiltinReference()) {
        return false;
    }
    
    outStartIdx = s_activeTrack.layout->builtinRefStartIdx;
    outEndIdx = s_activeTrack.layout->builtinRefEndIdx;
    outTimeMs = s_activeTrack.layout->builtinRefTimeMs;
    
    return true;
}

// ============================================================
// TRACK/LAYOUT ID ACCESS
// ============================================================

uint8_t getActiveTrackIndex() {
    if (!s_activeTrack.isValid()) return 0xFF;
    for (int i = 0; i < BUILTIN_TRACK_COUNT; i++) {
        if (BUILTIN_TRACKS[i] == s_activeTrack.track) return (uint8_t)i;
    }
    return 0xFF;
}

uint8_t getActiveTrackLayoutIndex() {
    if (!s_activeTrack.isValid()) return 0xFF;
    return (uint8_t)s_activeTrack.layoutIndex;
}

const char* getActiveTrackId() {
    if (!s_activeTrack.isValid()) return nullptr;
    return s_activeTrack.track->id;
}

const char* getActiveLayoutId() {
    if (!s_activeTrack.isValid() || !s_activeTrack.layout) return nullptr;
    return s_activeTrack.layout->id;
}

// ============================================================
// TRACK LISTING
// ============================================================

int getAvailableTrackCount() {
    return BUILTIN_TRACK_COUNT;
}

const TrackDefinition* getAvailableTrack(int index) {
    return getBuiltinTrack(index);
}

const char* getAvailableTrackName(int index) {
    const TrackDefinition* track = getBuiltinTrack(index);
    return track ? track->name : "Unknown";
}
