/**
 * @file gps_filter.cpp
 * @brief GPS data filtering and spike detection implementation
 * @version 1.0
 */

#include "gps_filter.h"
#include "geo_utils.h"

#include <algorithm>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPSFilter";

// ============================================================
// STATIC STATE
// ============================================================

static GPSFilterConfig s_config;
static GPSFilterState s_state;

// ============================================================
// INITIALIZATION
// ============================================================

void initGPSFilter(const GPSFilterConfig& config) {
    s_config = config;
    resetGPSFilter();
}

void resetGPSFilter() {
    s_state.lastValidPoint.clear();
    s_state.lastValidMs = 0;
    s_state.historyIndex = 0;
    s_state.historyCount = 0;
    s_state.totalPoints = 0;
    s_state.spikeCount = 0;
    s_state.smoothedCount = 0;
    s_state.signalLost = false;
    s_state.signalLostSinceMs = 0;
    
    // Clear history buffer
    for (int i = 0; i < GPSFilterState::MAX_HISTORY; i++) {
        s_state.history[i].clear();
    }
}

// ============================================================
// SPIKE DETECTION
// ============================================================

SpikeCheckResult checkForSpike(const GPSPoint& point, 
                                const GPSPoint& lastValid,
                                unsigned long elapsedMs) {
    SpikeCheckResult result;
    result.isSpike = false;
    result.confidence = 1.0f;
    
    // If no previous valid point, can't detect spike
    if (!lastValid.isValid()) {
        return result;
    }
    
    // Calculate distance from last valid point
    result.jumpDistanceM = fastDistanceMeters(
        lastValid.lat, lastValid.lng,
        point.lat, point.lng
    );
    
    // Calculate maximum expected distance based on speed and time
    // Use the faster of the two speeds to be conservative
    float maxSpeedKmh = std::max(lastValid.speedKmh, point.speedKmh);
    float maxSpeedMs = maxSpeedKmh * 1000.0f / 3600.0f;  // Convert to m/s
    float elapsedSec = elapsedMs / 1000.0f;
    
    // Expected max distance = speed * time * multiplier (for GPS jitter tolerance)
    result.expectedMaxM = maxSpeedMs * elapsedSec * s_config.jumpSpeedMultiplier;
    
    // Also enforce absolute maximum jump distance
    result.expectedMaxM = std::max(result.expectedMaxM, s_config.maxJumpDistanceM / 2.0f);
    
    // Check if jump exceeds threshold
    if (result.jumpDistanceM > s_config.maxJumpDistanceM) {
        // Absolute spike - too far regardless of speed
        result.isSpike = true;
        result.confidence = 0.0f;
    } else if (result.jumpDistanceM > result.expectedMaxM) {
        // Relative spike - too far for current speed
        result.isSpike = true;
        // Confidence decreases as jump exceeds expected
        float ratio = result.expectedMaxM / result.jumpDistanceM;
        result.confidence = std::clamp(ratio, 0.0f, 1.0f);
    } else {
        // Valid point
        result.isSpike = false;
        // Higher confidence when well within expected range
        float ratio = result.jumpDistanceM / result.expectedMaxM;
        result.confidence = 1.0f - (ratio * 0.3f);  // Small penalty for larger jumps
    }
    
    return result;
}

// ============================================================
// SMOOTHING
// ============================================================

GPSPoint smoothPoint(const GPSPoint& point) {
    if (!s_config.enableSmoothing || s_config.smoothingWindowSize <= 1) {
        return point;
    }
    
    // Add to history buffer
    s_state.history[s_state.historyIndex] = point;
    s_state.historyIndex = (s_state.historyIndex + 1) % GPSFilterState::MAX_HISTORY;
    if (s_state.historyCount < GPSFilterState::MAX_HISTORY) {
        s_state.historyCount++;
    }
    
    // Need at least 2 points to smooth
    int windowSize = std::min(s_config.smoothingWindowSize, s_state.historyCount);
    if (windowSize < 2) {
        return point;
    }
    
    // Calculate weighted moving average
    // More recent points get higher weight
    GPSPoint smoothed;
    double totalWeight = 0.0;
    double sumLat = 0.0;
    double sumLng = 0.0;
    float sumSpeed = 0.0f;

    // Sin/cos for heading averaging (to handle wrap-around)
    float sumSin = 0.0f;
    float sumCos = 0.0f;
    
    float weight = 1.0f;
    for (int i = 0; i < windowSize; i++) {
        // Get index into circular buffer (most recent first)
        int idx = (s_state.historyIndex - 1 - i + GPSFilterState::MAX_HISTORY) % GPSFilterState::MAX_HISTORY;
        const GPSPoint& p = s_state.history[idx];

        sumLat += p.lat * weight;
        sumLng += p.lng * weight;
        sumSpeed += p.speedKmh * weight;
        
        // Heading needs special handling for wrap-around
        float headingRad = p.headingDeg * GEO_DEG_TO_RAD;
        sumSin += sin(headingRad) * weight;
        sumCos += cos(headingRad) * weight;
        
        totalWeight += weight;
        weight *= 0.7f;
    }
    
    // Apply averages
    smoothed.lat = sumLat / totalWeight;
    smoothed.lng = sumLng / totalWeight;
    smoothed.speedKmh = sumSpeed / totalWeight;
    
    // Convert heading back from sin/cos
    smoothed.headingDeg = atan2(sumSin, sumCos) * GEO_RAD_TO_DEG;
    if (smoothed.headingDeg < 0) {
        smoothed.headingDeg += 360.0f;
    }
    
    // Keep original timestamps
    smoothed.gpsTimeMs = point.gpsTimeMs;
    smoothed.lapTimeMs = point.lapTimeMs;
    
    s_state.smoothedCount++;
    
    return smoothed;
}

// ============================================================
// MAIN FILTER FUNCTION
// ============================================================

FilteredGPSPoint filterGPSPoint(const GPSPoint& raw, unsigned long nowMs) {
    FilteredGPSPoint result;
    result.rawLat = (float)raw.lat;
    result.rawLng = (float)raw.lng;
    result.isValid = false;
    result.isInterpolated = false;
    result.wasSpikeFiltered = false;
    result.confidence = 0.0f;
    
    s_state.totalPoints++;
    
    // Check if raw point is valid at all
    if (!raw.isValid()) {
        // No valid GPS data
        if (s_state.lastValidMs > 0) {
            unsigned long elapsed = nowMs - s_state.lastValidMs;
            if (elapsed > s_config.signalLostThresholdMs && !s_state.signalLost) {
                s_state.signalLost = true;
                s_state.signalLostSinceMs = s_state.lastValidMs;
            }
        }
        return result;
    }
    
    // Calculate time since last valid point
    unsigned long elapsedMs = 0;
    if (s_state.lastValidMs > 0) {
        elapsedMs = nowMs - s_state.lastValidMs;
    }
    
    // Check for spike
    SpikeCheckResult spikeCheck = checkForSpike(raw, s_state.lastValidPoint, elapsedMs);
    
    if (spikeCheck.isSpike) {
        // Spike detected - reject this point
        result.wasSpikeFiltered = true;
        result.confidence = spikeCheck.confidence;
        s_state.spikeCount++;
        
        // Don't update last valid - keep waiting for good data
        // But return the last valid point as our "filtered" result
        result.point = s_state.lastValidPoint;
        result.isValid = s_state.lastValidPoint.isValid();
        result.isInterpolated = true;  // Not actually measured

        ESP_LOGD(TAG, "Spike rejected: jump=%.1fm, max=%.1fm",
                 spikeCheck.jumpDistanceM, spikeCheck.expectedMaxM);

        return result;
    }
    
    // Valid point - apply smoothing if enabled
    GPSPoint filtered = smoothPoint(raw);
    
    // Update state
    s_state.lastValidPoint = filtered;
    s_state.lastValidMs = nowMs;
    s_state.signalLost = false;
    
    // Build result
    result.point = filtered;
    result.isValid = true;
    result.confidence = spikeCheck.confidence;
    
    return result;
}

// ============================================================
// SIGNAL STATUS
// ============================================================

bool isGPSSignalLost() {
    return s_state.signalLost;
}

unsigned long getTimeSinceLastValid() {
    if (s_state.lastValidMs == 0) {
        return 0;
    }
    return (unsigned long)(esp_timer_get_time() / 1000ULL) - s_state.lastValidMs;
}

GPSPoint getLastValidPoint() {
    return s_state.lastValidPoint;
}

// ============================================================
// CONFIGURATION
// ============================================================

void setGPSFilterConfig(const GPSFilterConfig& config) {
    s_config = config;
}

GPSFilterConfig getGPSFilterConfig() {
    return s_config;
}

// ============================================================
// STATISTICS
// ============================================================

const GPSFilterState& getGPSFilterState() {
    return s_state;
}

float getSpikeRate() {
    if (s_state.totalPoints == 0) {
        return 0.0f;
    }
    return (float)s_state.spikeCount / (float)s_state.totalPoints * 100.0f;
}
