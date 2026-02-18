/**
 * @file track_manager.cpp
 * @brief Track detection and selection management implementation
 * @version 1.0
 */

#include "track_manager.h"
#include "../geo/geo_utils.h"
#include "config.h"

#include <cfloat>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "TrackMgr";

// ============================================================
// STATIC STATE
// ============================================================

static ActiveTrack s_activeTrack;
static bool s_initialized = false;

// Proximity state (separate from active track to avoid interference)
static bool s_isNearTrack = false;
static const TrackDefinition* s_proximityTrack = nullptr;

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
    s_isNearTrack = false;
    s_proximityTrack = nullptr;
}

// ============================================================
// TRACK DETECTION
// ============================================================

const TrackDefinition* detectTrackByPosition(double lat, double lng) {
    const TrackDefinition* bestTrack = nullptr;
    float bestDistance = FLT_MAX;
    
    // Search all built-in tracks
    for (int i = 0; i < BUILTIN_TRACK_COUNT; i++) {
        const TrackDefinition* track = BUILTIN_TRACKS[i];
        
        float distance = haversineDistanceMeters(
            lat, lng,
            track->centerLat, track->centerLng
        );
        
        // Check if within detection radius and closer than previous best
        if (distance <= track->detectionRadiusM && distance < bestDistance) {
            bestTrack = track;
            bestDistance = distance;
        }
    }
    
    // If we found a track and it's different from current (or no current)
    if (bestTrack != nullptr) {
        // If no active track or not user-confirmed, auto-select
        if (!s_activeTrack.isValid() || !s_activeTrack.userConfirmed) {
            setActiveTrack(bestTrack, 0);
            s_activeTrack.detectionDistanceM = bestDistance;
            s_activeTrack.detectedAtMs = (unsigned long)(esp_timer_get_time() / 1000ULL);
            
            ESP_LOGI(TAG, "Auto-detected: %s (%.0fm from center)",
                     bestTrack->name, bestDistance);
        }
    }
    
    return bestTrack;
}

bool isWithinActiveTrack(double lat, double lng) {
    if (!s_activeTrack.isValid()) {
        return false;
    }
    
    float distance = haversineDistanceMeters(
        lat, lng,
        s_activeTrack.track->centerLat,
        s_activeTrack.track->centerLng
    );
    
    return distance <= s_activeTrack.track->detectionRadiusM;
}

float getDistanceToNearestTrack(double lat, double lng, const TrackDefinition** outTrack) {
    const TrackDefinition* nearestTrack = nullptr;
    float nearestDistance = FLT_MAX;
    
    for (int i = 0; i < BUILTIN_TRACK_COUNT; i++) {
        const TrackDefinition* track = BUILTIN_TRACKS[i];
        
        float distance = haversineDistanceMeters(
            lat, lng,
            track->centerLat, track->centerLng
        );
        
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearestTrack = track;
        }
    }
    
    if (outTrack != nullptr) {
        *outTrack = nearestTrack;
    }
    
    return nearestDistance;
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
// PROXIMITY DETECTION (with hysteresis)
// ============================================================

bool updateTrackProximity(double lat, double lng) {
    // --- Hysteresis exit check: if already near, test at TRACK_PROXIMITY_HYSTERESIS_FACTOR× radius ---
    if (s_isNearTrack && s_proximityTrack != nullptr) {
        float exitRadius = s_proximityTrack->detectionRadiusM * TRACK_PROXIMITY_HYSTERESIS_FACTOR;

        // Stage 1: bounding box pre-filter (METERS_PER_DEG_LAT ≈ 111320; use same for lng as pre-filter)
        float dlat_m = (float)((lat - s_proximityTrack->centerLat) * METERS_PER_DEG_LAT);
        float dlng_m = (float)((lng - s_proximityTrack->centerLng) * METERS_PER_DEG_LAT);
        if (dlat_m < 0) dlat_m = -dlat_m;
        if (dlng_m < 0) dlng_m = -dlng_m;

        bool outsideBbox = (dlat_m > exitRadius || dlng_m > exitRadius);
        if (outsideBbox) {
            ESP_LOGI(TAG, "Left proximity of %s (bbox)", s_proximityTrack->name);
            s_isNearTrack = false;
            s_proximityTrack = nullptr;
        } else {
            // Stage 2: precise haversine
            float dist = haversineDistanceMeters(lat, lng,
                s_proximityTrack->centerLat, s_proximityTrack->centerLng);
            if (dist >= exitRadius) {
                ESP_LOGI(TAG, "Left proximity of %s (%.0fm >= %.0fm)",
                         s_proximityTrack->name, dist, exitRadius);
                s_isNearTrack = false;
                s_proximityTrack = nullptr;
            }
        }

        if (s_isNearTrack) return true;  // Still near, skip scan
    }

    // --- Entry check: scan all tracks at 1× radius ---
    for (int i = 0; i < BUILTIN_TRACK_COUNT; i++) {
        const TrackDefinition* track = BUILTIN_TRACKS[i];
        float radius = track->detectionRadiusM;

        // Stage 1: cheap bounding box pre-filter
        float dlat_m = (float)((lat - track->centerLat) * METERS_PER_DEG_LAT);
        float dlng_m = (float)((lng - track->centerLng) * METERS_PER_DEG_LAT);
        if (dlat_m < 0) dlat_m = -dlat_m;
        if (dlng_m < 0) dlng_m = -dlng_m;
        if (dlat_m > radius || dlng_m > radius) continue;

        // Stage 2: haversine
        float dist = haversineDistanceMeters(lat, lng,
            track->centerLat, track->centerLng);
        if (dist <= radius) {
            s_isNearTrack = true;
            s_proximityTrack = track;
            ESP_LOGI(TAG, "Near track: %s (%.0fm)", track->name, dist);

            // Auto-select active track if none confirmed
            if (!s_activeTrack.isValid() || !s_activeTrack.userConfirmed) {
                setActiveTrack(track, 0);
                s_activeTrack.detectionDistanceM = dist;
                s_activeTrack.detectedAtMs = (unsigned long)(esp_timer_get_time() / 1000ULL);
            }
            return true;
        }
    }

    return false;
}

bool isNearAnyTrack() {
    return s_isNearTrack;
}

const char* getNearTrackName() {
    if (s_isNearTrack && s_proximityTrack != nullptr) {
        return s_proximityTrack->name;
    }
    return nullptr;
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
