/**
 * @file lap_manager.cpp
 * @brief Lap and reference lap management implementation
 * @version 1.0
 */

#include "lap_manager.h"
#include "delta_calculator.h"
#include "../track/track_manager.h"
#include "../track/builtin_tracks.h"
#include "../geo/geo_utils.h"

#include <climits>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// STATIC STATE
// ============================================================

static LapState s_lapState;
static uint16_t s_bestLapNumber = 0;
static bool s_initialized = false;

// Lap recording
static LapData s_recordingLap;
static bool s_isRecording = false;

// ============================================================
// INITIALIZATION
// ============================================================

void initLapManager() {
    s_lapState.clear();
    gApp.bestLapTimeMs = UINT32_MAX;
    gApp.currentLapNumber = 1;
    gApp.currentSessionNumber = 1;
    gApp.currentPoint.clear();
    gApp.previousPoint.clear();
    s_bestLapNumber = 0;
    gApp.hasValidReferenceLap = false;
    s_isRecording = false;
    s_recordingLap.clear();
    // Pre-reserve to prevent heap fragmentation during lap recording
    s_recordingLap.points.reserve(MAX_REFERENCE_POINTS);
    s_recordingLap.cumulativeDistances.reserve(MAX_REFERENCE_POINTS);
    gApp.referenceLap.points.reserve(MAX_REFERENCE_POINTS);
    gApp.referenceLap.cumulativeDistances.reserve(MAX_REFERENCE_POINTS);
    s_initialized = true;

    // Initialize top 3 laps display
    for (int i = 0; i < 3; i++) {
        gApp.top3Laps[i].lapNumber = 0;
        gApp.top3Laps[i].lapTimeMs = UINT32_MAX;
    }

    printf("[LapManager] Initialized\n");
}

void resetLapManager() {
    s_lapState.clear();
    s_isRecording = false;
    s_recordingLap.clear();
    
    // Preserve best lap and reference - they persist
}

// ============================================================
// SESSION MANAGEMENT
// ============================================================

uint16_t startNewSession() {
    gApp.currentSessionNumber++;
    gApp.currentLapNumber = 1;
    s_lapState.lapInProgress = false;
    
    // Clear top 3 laps for new session
    for (int i = 0; i < 3; i++) {
        gApp.top3Laps[i].lapNumber = 0;
        gApp.top3Laps[i].lapTimeMs = UINT32_MAX;
    }
    
    printf("[LapManager] New session: %d\n", gApp.currentSessionNumber);
    
    return gApp.currentSessionNumber;
}

uint16_t getCurrentSession() {
    return gApp.currentSessionNumber;
}

void setCurrentSession(uint16_t session) {
    gApp.currentSessionNumber = session;
}

// ============================================================
// LAP CONTROL
// ============================================================

void startLap(unsigned long startTimeMs) {
    s_lapState.lapInProgress = true;
    s_lapState.lapStartMs = startTimeMs;
    s_lapState.currentLapTimeMs = 0;
    
    // Clear point history for new lap
    gApp.previousPoint.clear();
    
    // Reset delta calculator for new lap
    resetDeltaCalculator();
    
    printf("[LapManager] Lap %d started\n", gApp.currentLapNumber);
}

unsigned long completeLap(unsigned long finishTimeMs) {
    if (!s_lapState.lapInProgress) {
        return 0;
    }
    
    unsigned long lapTime = finishTimeMs - s_lapState.lapStartMs;
    
    s_lapState.lapInProgress = false;
    s_lapState.lastLapTimeMs = lapTime;
    s_lapState.currentLapTimeMs = lapTime;
    
    // Check for new best
    s_lapState.lastLapWasBest = isNewBestLap(lapTime);
    if (s_lapState.lastLapWasBest) {
        gApp.bestLapTimeMs = lapTime;
        s_bestLapNumber = gApp.currentLapNumber;
        
        // First lap completion means we now have a valid reference
        if (!gApp.hasValidReferenceLap) {
            gApp.hasValidReferenceLap = true;
        }
        
        printf("[LapManager] NEW BEST LAP: %.3fs\n", lapTime / 1000.0f);
    }
    
    printf("[LapManager] Lap %d complete: %.3fs\n", 
                 gApp.currentLapNumber, lapTime / 1000.0f);
    
    return lapTime;
}

void cancelLap() {
    s_lapState.lapInProgress = false;
    s_lapState.currentLapTimeMs = 0;
    
    if (s_isRecording) {
        cancelLapRecording();
    }
    
    printf("[LapManager] Lap cancelled\n");
}

bool isLapInProgress() {
    return s_lapState.lapInProgress;
}

unsigned long getLapElapsedTime(unsigned long nowMs) {
    if (!s_lapState.lapInProgress) {
        return 0;
    }
    
    return nowMs - s_lapState.lapStartMs;
}

// ============================================================
// LAP NUMBER MANAGEMENT
// ============================================================

uint16_t getCurrentLapNumber() {
    return gApp.currentLapNumber;
}

void setCurrentLapNumber(uint16_t lap) {
    gApp.currentLapNumber = lap;
}

void incrementLapNumber() {
    gApp.currentLapNumber++;
}

// ============================================================
// GPS POINT TRACKING
// ============================================================

void updateCurrentPoint(const GPSPoint& point) {
    gApp.previousPoint = gApp.currentPoint;
    gApp.currentPoint = point;
}

const GPSPoint& getCurrentPoint() {
    return gApp.currentPoint;
}

const GPSPoint& getPreviousPoint() {
    return gApp.previousPoint;
}

void clearPointHistory() {
    gApp.currentPoint.clear();
    gApp.previousPoint.clear();
}

// ============================================================
// BEST LAP MANAGEMENT
// ============================================================

uint32_t getBestLapTimeMs() {
    return gApp.bestLapTimeMs;
}

void setBestLapTimeMs(uint32_t timeMs) {
    gApp.bestLapTimeMs = timeMs;
}

bool isNewBestLap(uint32_t timeMs) {
    if (timeMs == 0) {
        return false;
    }
    return timeMs < gApp.bestLapTimeMs;
}

uint16_t getBestLapNumber() {
    return s_bestLapNumber;
}

bool hasBestLapTime() {
    return gApp.bestLapTimeMs < UINT32_MAX;
}

// ============================================================
// REFERENCE LAP MANAGEMENT
// ============================================================

bool hasValidReferenceLap() {
    return gApp.hasValidReferenceLap && hasReferenceLap();
}

void setHasValidReferenceLap(bool valid) {
    gApp.hasValidReferenceLap = valid;
}

bool loadRefLapFromStorage() {
    // TODO: Implement SPIFFS loading
    // This would load from lap_storage module
    printf("[LapManager] loadRefLapFromStorage - not implemented\n");
    return false;
}

bool loadRefLapFromTrackData(int lapIdx) {
    // Get track data
    const LapBoundary* boundaries = getEverlandLapBoundaries();
    const TrackPoint* points = getEverlandTrackPoints();
    int lapCount = getEverlandLapCount();
    
    if (lapIdx < 0 || lapIdx >= lapCount) {
        printf("[LapManager] Invalid lap index: %d\n", lapIdx);
        return false;
    }
    
    const LapBoundary& bounds = boundaries[lapIdx];
    
    // Build LapData from track points
    LapData lap;
    lap.points.clear();
    lap.points.reserve(bounds.endIndex - bounds.startIndex + 1);
    
    float maxSpeed = 0.0f;
    float totalSpeed = 0.0f;
    
    for (int i = bounds.startIndex; i <= bounds.endIndex; i++) {
        const TrackPoint& tp = points[i];
        
        GPSPoint point;
        point.lat = tp.lat / 1e7;
        point.lng = tp.lon / 1e7;
        point.lapTimeMs = tp.timeMs;
        point.speedKmh = tp.speedX10 / 10.0f;
        point.headingDeg = 0.0f;
        point.gpsTimeMs = tp.timeMs;
        
        lap.points.push_back(point);
        
        totalSpeed += point.speedKmh;
        if (point.speedKmh > maxSpeed) {
            maxSpeed = point.speedKmh;
        }
        
        // Yield periodically for large datasets
        if (i % 100 == 0) {
            taskYIELD();
        }
    }
    
    if (lap.points.empty()) {
        return false;
    }
    
    lap.startTimeMs = 0;
    lap.totalTimeMs = bounds.lapTimeMs;
    lap.maxSpeedKmh = maxSpeed;
    lap.avgSpeedKmh = totalSpeed / lap.points.size();
    
    // Set as reference lap
    if (!setReferenceLap(lap)) {
        return false;
    }
    
    gApp.bestLapTimeMs = bounds.lapTimeMs;
    gApp.hasValidReferenceLap = true;
    
    printf("[LapManager] Loaded track data lap %d: %d pts, %.2fs\n",
                 lapIdx, lap.points.size(), lap.totalTimeMs / 1000.0f);
    
    return true;
}

bool loadReferenceLapFromPriorSession() {
    // Load from embedded reference data (everland_reference_5283)
    const auto* refPoints = getEverlandReferencePoints();
    int refCount = getEverlandReferencePointCount();
    
    if (refPoints == nullptr || refCount <= 0) {
        return false;
    }
    
    LapData lap;
    lap.points.clear();
    lap.points.reserve(refCount);
    
    float maxSpeed = 0.0f;
    float totalSpeed = 0.0f;
    
    for (int i = 0; i < refCount; i++) {
        const auto& tp = refPoints[i];
        
        GPSPoint point;
        point.lat = tp.lat / 1e7;
        point.lng = tp.lon / 1e7;
        point.lapTimeMs = tp.timeMs;
        point.speedKmh = tp.speedX10 / 10.0f;
        point.headingDeg = 0.0f;
        point.gpsTimeMs = tp.timeMs;
        
        lap.points.push_back(point);
        
        totalSpeed += point.speedKmh;
        if (point.speedKmh > maxSpeed) {
            maxSpeed = point.speedKmh;
        }
    }
    
    if (lap.points.empty()) {
        return false;
    }
    
    lap.startTimeMs = 0;
    // Get lap time from the last point
    lap.totalTimeMs = lap.points.back().lapTimeMs;
    lap.maxSpeedKmh = maxSpeed;
    lap.avgSpeedKmh = totalSpeed / lap.points.size();
    
    // Set as reference lap
    if (!setReferenceLap(lap)) {
        return false;
    }
    
    gApp.bestLapTimeMs = lap.totalTimeMs;
    gApp.hasValidReferenceLap = true;
    
    printf("[LapManager] Loaded prior session reference: %d pts, %.2fs\n",
                 lap.points.size(), lap.totalTimeMs / 1000.0f);
    
    return true;
}

bool saveAsReferenceLap(const LapData& lap) {
    if (!lap.isValid()) {
        return false;
    }
    
    // Set as current reference
    if (!setReferenceLap(lap)) {
        return false;
    }
    
    gApp.hasValidReferenceLap = true;
    
    // TODO: Also save to SPIFFS via lap_storage
    
    printf("[LapManager] Saved as reference: %d pts, %.2fs\n",
                 lap.points.size(), lap.totalTimeMs / 1000.0f);
    
    return true;
}

// ============================================================
// LAP STATE ACCESS
// ============================================================

const LapState& getLapState() {
    return s_lapState;
}

LapState& getLapStateMutable() {
    return s_lapState;
}

// ============================================================
// LAP RECORDING
// ============================================================

void startLapRecording() {
    s_recordingLap.clear();
    s_isRecording = true;
    
    printf("[LapManager] Started recording\n");
}

void recordLapPoint(double lat, double lng, unsigned long lapTimeMs,
                    float speedKmh, float headingDeg) {
    if (!s_isRecording) {
        return;
    }
    
    GPSPoint point;
    point.lat = lat;
    point.lng = lng;
    point.lapTimeMs = lapTimeMs;
    point.speedKmh = speedKmh;
    point.headingDeg = headingDeg;
    point.gpsTimeMs = 0;
    
    s_recordingLap.points.push_back(point);
    
    // Update statistics
    if (speedKmh > s_recordingLap.maxSpeedKmh) {
        s_recordingLap.maxSpeedKmh = speedKmh;
    }
}

bool finishLapRecording(LapData& outLap) {
    if (!s_isRecording || s_recordingLap.points.empty()) {
        s_isRecording = false;
        return false;
    }
    
    // Finalize lap data
    s_recordingLap.totalTimeMs = s_recordingLap.points.back().lapTimeMs;
    
    // Calculate average speed
    float totalSpeed = 0.0f;
    for (const auto& pt : s_recordingLap.points) {
        totalSpeed += pt.speedKmh;
    }
    s_recordingLap.avgSpeedKmh = totalSpeed / s_recordingLap.points.size();
    
    // Calculate cumulative distances
    calculateCumulativeDistances(s_recordingLap);
    
    outLap = s_recordingLap;
    s_isRecording = false;
    s_recordingLap.clear();
    
    printf("[LapManager] Recording finished: %d pts, %.2fs\n",
                 outLap.points.size(), outLap.totalTimeMs / 1000.0f);
    
    return true;
}

void cancelLapRecording() {
    s_isRecording = false;
    s_recordingLap.clear();
    
    printf("[LapManager] Recording cancelled\n");
}

bool isRecordingLap() {
    return s_isRecording;
}

// ============================================================
// LAP VALIDATION
// ============================================================

bool isValidLapTime(unsigned long timeMs) {
    return timeMs >= getMinLapTimeMs() && timeMs <= getMaxLapTimeMs();
}

unsigned long getMinLapTimeMs() {
    // Check if we have active track with min lap time
    if (hasActiveTrack()) {
        const ActiveTrack& active = getActiveTrackConst();
        if (active.layout != nullptr) {
            return active.layout->minLapTimeMs;
        }
    }
    
    // Default minimum
    return MIN_LAP_TIME_MS;
}

unsigned long getMaxLapTimeMs() {
    // Check if we have active track with max lap time
    if (hasActiveTrack()) {
        const ActiveTrack& active = getActiveTrackConst();
        if (active.layout != nullptr) {
            return active.layout->maxLapTimeMs;
        }
    }
    
    // Default maximum (10 minutes)
    return 600000;
}