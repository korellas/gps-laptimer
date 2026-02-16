/**
 * @file dead_reckoning.cpp
 * @brief Dead reckoning for GPS signal loss implementation
 * @version 1.0
 */

#include "dead_reckoning.h"
#include "geo_utils.h"

#include <algorithm>
#include <cstdio>

#include "esp_log.h"

static const char *TAG = "DeadReckoning";

// ============================================================
// STATIC STATE
// ============================================================

static DRConfig s_config;
static DRState s_state;

// ============================================================
// INITIALIZATION
// ============================================================

void initDeadReckoning(const DRConfig& config) {
    s_config = config;
    resetDeadReckoning();
}

void resetDeadReckoning() {
    s_state.active = false;
    s_state.startMs = 0;
    s_state.lastUpdateMs = 0;
    s_state.totalDurationMs = 0;
    s_state.lastKnownPoint.clear();
    s_state.lastSpeedKmh = 0.0f;
    s_state.lastHeadingDeg = 0.0f;
    s_state.estimatedPoint.clear();
    s_state.confidence = 1.0f;
    s_state.estimatedErrorM = 0.0f;
}

// ============================================================
// DEAD RECKONING CONTROL
// ============================================================

bool startDeadReckoning(const GPSPoint& lastPoint, 
                        float speedKmh, 
                        float headingDeg,
                        unsigned long nowMs) {
    // Check if DR is enabled
    if (!s_config.enabled) {
        return false;
    }
    
    // Check minimum speed requirement
    if (speedKmh < s_config.minSpeedKmh) {
        ESP_LOGD(TAG, "Not starting: speed %.1f < min %.1f km/h", speedKmh, s_config.minSpeedKmh);
        return false;
    }

    // Check if last point is valid
    if (!lastPoint.isValid()) {
        ESP_LOGD(TAG, "Not starting: invalid last point");
        return false;
    }
    
    // Start DR
    s_state.active = true;
    s_state.startMs = nowMs;
    s_state.lastUpdateMs = nowMs;
    s_state.totalDurationMs = 0;
    
    s_state.lastKnownPoint = lastPoint;
    s_state.lastSpeedKmh = speedKmh;
    s_state.lastHeadingDeg = headingDeg;
    
    // Initial estimate is the last known position
    s_state.estimatedPoint = lastPoint;
    s_state.confidence = 1.0f;
    s_state.estimatedErrorM = 0.0f;

    ESP_LOGD(TAG, "Started: lat=%.6f, lng=%.6f, speed=%.1f km/h, heading=%.1f°",
             lastPoint.lat, lastPoint.lng, speedKmh, headingDeg);

    return true;
}

GPSPoint updateDeadReckoning(unsigned long nowMs) {
    // Return empty point if not active
    if (!s_state.active) {
        GPSPoint empty;
        return empty;
    }
    
    // Calculate elapsed time since start
    unsigned long elapsedSinceStart = nowMs - s_state.startMs;
    
    // Check if expired
    if (elapsedSinceStart > s_config.maxDurationMs) {
        ESP_LOGD(TAG, "Expired after %lu ms", elapsedSinceStart);
        // Don't stop automatically - let caller decide
        // Just return last estimate with low confidence
        s_state.confidence = 0.0f;
        return s_state.estimatedPoint;
    }
    
    // Update total duration
    s_state.totalDurationMs = elapsedSinceStart;
    
    // Estimate new position from last known point
    // (Always calculate from last known, not from previous estimate, to avoid error accumulation)
    double newLat, newLng;
    estimatePosition(
        s_state.lastKnownPoint.lat,
        s_state.lastKnownPoint.lng,
        s_state.lastSpeedKmh,
        s_state.lastHeadingDeg,
        elapsedSinceStart,
        newLat, newLng
    );
    
    // Update estimated point
    s_state.estimatedPoint.lat = newLat;
    s_state.estimatedPoint.lng = newLng;
    s_state.estimatedPoint.speedKmh = s_state.lastSpeedKmh;  // Assume constant speed
    s_state.estimatedPoint.headingDeg = s_state.lastHeadingDeg;  // Assume constant heading
    s_state.estimatedPoint.gpsTimeMs = nowMs;
    
    // Update lap time if original had one
    if (s_state.lastKnownPoint.lapTimeMs > 0) {
        s_state.estimatedPoint.lapTimeMs = s_state.lastKnownPoint.lapTimeMs + elapsedSinceStart;
    }
    
    // Calculate confidence decay
    float elapsedSec = elapsedSinceStart / 1000.0f;
    s_state.confidence = 1.0f - (elapsedSec * s_config.confidenceDecayPerSec);
    s_state.confidence = std::clamp(s_state.confidence, 0.0f, 1.0f);
    
    // Calculate estimated error
    s_state.estimatedErrorM = elapsedSec * s_config.errorGrowthMPerSec;
    
    // Update last update time
    s_state.lastUpdateMs = nowMs;

    if ((elapsedSinceStart % 1000) < 50) {  // Log ~once per second
        ESP_LOGD(TAG, "Update: lat=%.6f, lng=%.6f, conf=%.0f%%, err=%.1fm",
                 s_state.estimatedPoint.lat,
                 s_state.estimatedPoint.lng,
                 s_state.confidence * 100.0f,
                 s_state.estimatedErrorM);
    }

    return s_state.estimatedPoint;
}

void stopDeadReckoning() {
    if (s_state.active) {
        ESP_LOGD(TAG, "Stopped after %lu ms", s_state.totalDurationMs);
    }

    s_state.active = false;
}

// ============================================================
// STATUS QUERIES
// ============================================================

bool isDeadReckoningActive() {
    return s_state.active;
}

bool isDeadReckoningExpired() {
    if (!s_state.active) {
        return false;
    }
    return s_state.totalDurationMs > s_config.maxDurationMs;
}

const DRState& getDeadReckoningState() {
    return s_state;
}

float getDeadReckoningConfidence() {
    if (!s_state.active) {
        return 0.0f;
    }
    return s_state.confidence;
}

float getDeadReckoningError() {
    if (!s_state.active) {
        return 0.0f;
    }
    return s_state.estimatedErrorM;
}

// ============================================================
// CONFIGURATION
// ============================================================

void setDRConfig(const DRConfig& config) {
    s_config = config;
}

DRConfig getDRConfig() {
    return s_config;
}