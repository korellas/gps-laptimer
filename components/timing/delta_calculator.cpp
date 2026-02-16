/**
 * @file delta_calculator.cpp
 * @brief Delta time calculation implementation
 * @version 1.0
 */

#include "delta_calculator.h"
#include "../geo/geo_utils.h"
#include "config.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>

// cos(lat) 사전계산 캐시 (트랙 위도 기준, 세그먼트 검색에 재사용)
static float s_cosLat = 0.0f;

// ============================================================
// STATIC STATE
// ============================================================

static DeltaConfig s_config;
static float s_currentTrackDistance = -1.0f;
static bool s_initialized = false;

// ============================================================
// INITIALIZATION
// ============================================================

void initDeltaCalculator(const DeltaConfig& config) {
    s_config = config;
    resetDeltaCalculator();
    s_initialized = true;
    
    printf("[DeltaCalc] Initialized\n");
}

void resetDeltaCalculator() {
    gApp.lastValidSegmentIndex = -1;
    s_currentTrackDistance = -1.0f;
}

// ============================================================
// REFERENCE LAP MANAGEMENT
// ============================================================

bool setReferenceLap(const LapData& lap) {
    if (lap.points.empty()) {
        printf("[DeltaCalc] Cannot set empty reference lap\n");
        return false;
    }

    gApp.referenceLap = lap;

    // Pre-calculate cumulative distances if not already done
    if (gApp.referenceLap.cumulativeDistances.empty()) {
        calculateCumulativeDistances(gApp.referenceLap);
    }

    gApp.hasValidReferenceLap = true;
    gApp.lastValidSegmentIndex = -1;

    printf("[DeltaCalc] Reference set: %d pts, %.2fs, %.1fm\n",
           (int)gApp.referenceLap.points.size(),
           gApp.referenceLap.totalTimeMs / 1000.0f,
           gApp.referenceLap.totalTrackDistance);

    return true;
}

bool hasReferenceLap() {
    return gApp.hasValidReferenceLap && !gApp.referenceLap.points.empty();
}

const LapData& getReferenceLap() {
    return gApp.referenceLap;
}

void clearReferenceLap() {
    gApp.referenceLap.clear();
    gApp.hasValidReferenceLap = false;
    gApp.lastValidSegmentIndex = -1;

    printf("[DeltaCalc] Reference cleared\n");
}

unsigned long getReferenceLapTimeMs() {
    return gApp.hasValidReferenceLap ? gApp.referenceLap.totalTimeMs : 0;
}

// ============================================================
// PRE-CALCULATION
// ============================================================

void calculateCumulativeDistances(LapData& lap) {
    lap.cumulativeDistances.clear();
    lap.cumulativeDistances.reserve(lap.points.size());
    
    float cumDist = 0.0f;
    lap.cumulativeDistances.push_back(0.0f);  // First point at distance 0
    
    // cos(lat) 한 번 계산 후 전체 세그먼트에 재사용
    if (!lap.points.empty()) {
        double avgLat = lap.points[0].lat * GEO_DEG_TO_RAD;
        s_cosLat = (float)cos(avgLat);
    }

    for (size_t i = 1; i < lap.points.size(); i++) {
        float segDist = fastDistanceMetersPrecomp(
            lap.points[i-1].lat, lap.points[i-1].lng,
            lap.points[i].lat, lap.points[i].lng,
            s_cosLat
        );
        cumDist += segDist;
        lap.cumulativeDistances.push_back(cumDist);
    }
    
    lap.totalTrackDistance = cumDist;
    
    printf("[DeltaCalc] Track distance: %.1fm (%d pts)\n",
           cumDist, lap.points.size());
}

// ============================================================
// SPEED DELTA HELPERS
// ============================================================

static bool isValidSpeedKmh(float speedKmh) {
    return std::isfinite(speedKmh) && speedKmh >= 1.0f && speedKmh <= 450.0f;
}

static float estimateSpeedFromPointsKmh(const GPSPoint& prev, const GPSPoint& cur) {
    if (!prev.isValid() || !cur.isValid()) {
        return NAN;
    }
    if (cur.lapTimeMs <= prev.lapTimeMs) {
        return NAN;
    }
    unsigned long dtMs = cur.lapTimeMs - prev.lapTimeMs;
    if (dtMs < 20) {
        return NAN;
    }
    float distM = fastDistanceMeters(prev.lat, prev.lng, cur.lat, cur.lng);
    float speedKmh = (distM / (dtMs / 1000.0f)) * 3.6f;
    return isValidSpeedKmh(speedKmh) ? speedKmh : NAN;
}

static float getCurrentSpeedForDeltaKmh(const GPSPoint& current, const GPSPoint* previousPoint) {
    if (isValidSpeedKmh(current.speedKmh)) {
        return current.speedKmh;
    }
    if (previousPoint) {
        return estimateSpeedFromPointsKmh(*previousPoint, current);
    }
    return NAN;
}

// ============================================================
// DELTA CALCULATION
// ============================================================

DeltaResult calculateDelta(const GPSPoint& current, const LapData& reference,
                           const GPSPoint* previousPoint) {
    DeltaResult result;
    result.distanceMeters = FLT_MAX;

    if (reference.points.size() < 2 || reference.cumulativeDistances.empty()) {
        return result;
    }

    // Find the closest segment with continuity preference
    float minDistanceM = FLT_MAX;
    int bestSegmentStart = -1;
    float bestT = 0.0f;

    int searchStart = 0;
    int searchEnd = (int)reference.points.size() - 1;

    // cos(lat) 사전계산 — 세그먼트 검색 전 한 번만 계산
    float cosLat = s_cosLat;
    if (cosLat == 0.0f) {
        cosLat = (float)cos(current.lat * GEO_DEG_TO_RAD);
    }

    auto doSearch = [&](int start, int end) {
        for (int i = start; i < end; i++) {
            const GPSPoint& p1 = reference.points[i];
            const GPSPoint& p2 = reference.points[i + 1];

            float t = projectToSegment(current.lat, current.lng, p1.lat, p1.lng, p2.lat, p2.lng);

            double projLat = p1.lat + t * (p2.lat - p1.lat);
            double projLng = p1.lng + t * (p2.lng - p1.lng);

            float dist = fastDistanceMetersPrecomp(current.lat, current.lng, projLat, projLng, cosLat);

            if (dist < minDistanceM) {
                minDistanceM = dist;
                bestSegmentStart = i;
                bestT = t;
            }
        }
    };

    if (gApp.lastValidSegmentIndex >= 0) {
        // Search in window around last valid position
        int windowSize = SEGMENT_SEARCH_WINDOW;
        searchStart = (gApp.lastValidSegmentIndex - SEGMENT_SEARCH_BACK > 0) ?
                      gApp.lastValidSegmentIndex - SEGMENT_SEARCH_BACK : 0;
        searchEnd = (gApp.lastValidSegmentIndex + windowSize < (int)reference.points.size() - 1) ?
                    gApp.lastValidSegmentIndex + windowSize : (int)reference.points.size() - 1;

        doSearch(searchStart, searchEnd);

        // Fallback to full search if no good match
        if (minDistanceM >= MAX_PROJECTION_DISTANCE_M) {
            minDistanceM = FLT_MAX;
            bestSegmentStart = -1;
            doSearch(0, (int)reference.points.size() - 1);
        }
    } else {
        // No previous segment - search entire track
        doSearch(0, (int)reference.points.size() - 1);
    }

    result.distanceMeters = minDistanceM;
    result.refPointIndex = bestSegmentStart;

    if (bestSegmentStart < 0 || minDistanceM >= MAX_PROJECTION_DISTANCE_M) {
        return result;
    }

    gApp.lastValidSegmentIndex = bestSegmentStart;

    // Calculate track distance
    float segmentStartDist = reference.cumulativeDistances[bestSegmentStart];
    float segmentEndDist = reference.cumulativeDistances[bestSegmentStart + 1];
    float segmentLength = segmentEndDist - segmentStartDist;

    float currentTrackDistance = segmentStartDist + (bestT * segmentLength);
    result.trackDistanceM = currentTrackDistance;
    s_currentTrackDistance = currentTrackDistance;

    // Interpolate reference time
    const GPSPoint& p1 = reference.points[bestSegmentStart];
    const GPSPoint& p2 = reference.points[bestSegmentStart + 1];

    float refTimeSec = (p1.lapTimeMs + bestT * (p2.lapTimeMs - p1.lapTimeMs)) / 1000.0f;
    result.refTimeSec = refTimeSec;

    // Calculate time delta
    float curTimeSec = current.lapTimeMs / 1000.0f;
    result.deltaSeconds = curTimeSec - refTimeSec;

    // Calculate speed delta at same track position
    float refSpeed1 = p1.speedKmh;
    float refSpeed2 = p2.speedKmh;
    float segmentTimeSec = (p2.lapTimeMs > p1.lapTimeMs) ? ((p2.lapTimeMs - p1.lapTimeMs) / 1000.0f) : 0.0f;
    float segmentDerivedSpeed = (segmentTimeSec > 0.02f) ? ((segmentLength / segmentTimeSec) * 3.6f) : NAN;
    if (!isValidSpeedKmh(refSpeed1)) {
        refSpeed1 = segmentDerivedSpeed;
    }
    if (!isValidSpeedKmh(refSpeed2)) {
        refSpeed2 = segmentDerivedSpeed;
    }

    float refSpeedKmh = NAN;
    if (isValidSpeedKmh(refSpeed1) && isValidSpeedKmh(refSpeed2)) {
        refSpeedKmh = refSpeed1 + bestT * (refSpeed2 - refSpeed1);
    } else if (isValidSpeedKmh(refSpeed1)) {
        refSpeedKmh = refSpeed1;
    } else if (isValidSpeedKmh(refSpeed2)) {
        refSpeedKmh = refSpeed2;
    }

    float curSpeedKmh = getCurrentSpeedForDeltaKmh(current, previousPoint);
    if (isValidSpeedKmh(curSpeedKmh) && isValidSpeedKmh(refSpeedKmh)) {
        result.refSpeedKmh = refSpeedKmh;
        result.speedDeltaKmh = curSpeedKmh - refSpeedKmh;
        result.hasSpeedDelta = true;
    }

    // Confidence based on distance from track
    result.confidence = expf(-CONFIDENCE_DECAY_FACTOR * minDistanceM / MAX_PROJECTION_DISTANCE_M);

    return result;
}

DeltaResult calculateDeltaAtDistance(float trackDistanceM, unsigned long currentLapTimeMs) {
    DeltaResult result;
    result.clear();
    
    if (!gApp.hasValidReferenceLap || gApp.referenceLap.cumulativeDistances.empty()) {
        return result;
    }
    
    result.trackDistanceM = trackDistanceM;
    
    // Find reference time at this distance
    unsigned long refTimeMs = getReferenceTimeAtDistance(trackDistanceM);
    if (refTimeMs == 0) {
        return result;
    }
    
    result.refTimeSec = refTimeMs / 1000.0f;
    result.deltaSeconds = (float)(currentLapTimeMs - refTimeMs) / 1000.0f;
    result.confidence = 1.0f;  // High confidence when using known distance
    
    return result;
}

unsigned long getReferenceTimeAtDistance(float trackDistanceM) {
    if (!gApp.hasValidReferenceLap || gApp.referenceLap.cumulativeDistances.empty()) {
        return 0;
    }
    
    const auto& cumDist = gApp.referenceLap.cumulativeDistances;
    const auto& refPoints = gApp.referenceLap.points;
    
    // Clamp to valid range
    if (trackDistanceM <= 0) {
        return refPoints[0].lapTimeMs;
    }
    if (trackDistanceM >= gApp.referenceLap.totalTrackDistance) {
        return refPoints.back().lapTimeMs;
    }
    
    // Binary search for segment containing this distance
    int left = 0;
    int right = (int)cumDist.size() - 1;
    
    while (left < right - 1) {
        int mid = (left + right) / 2;
        if (cumDist[mid] <= trackDistanceM) {
            left = mid;
        } else {
            right = mid;
        }
    }
    
    // Interpolate time within segment
    float segStart = cumDist[left];
    float segEnd = cumDist[right];
    float segLength = segEnd - segStart;
    
    if (segLength < 0.001f) {
        return refPoints[left].lapTimeMs;
    }
    
    float t = (trackDistanceM - segStart) / segLength;
    t = std::clamp(t, 0.0f, 1.0f);
    
    unsigned long time1 = refPoints[left].lapTimeMs;
    unsigned long time2 = refPoints[right].lapTimeMs;
    
    return time1 + (unsigned long)(t * (time2 - time1));
}

// ============================================================
// SEGMENT TRACKING
// ============================================================

int getLastMatchedSegment() {
    return gApp.lastValidSegmentIndex;
}

void setLastMatchedSegment(int segmentIndex) {
    gApp.lastValidSegmentIndex = segmentIndex;
}

// ============================================================
// TRACK PROGRESS
// ============================================================

float getTrackProgress() {
    if (!gApp.hasValidReferenceLap || gApp.referenceLap.totalTrackDistance <= 0) {
        return -1.0f;
    }
    
    if (s_currentTrackDistance < 0) {
        return -1.0f;
    }
    
    return s_currentTrackDistance / gApp.referenceLap.totalTrackDistance;
}

float getCurrentTrackDistance() {
    return s_currentTrackDistance;
}

// ============================================================
// CONFIGURATION
// ============================================================

void setDeltaConfig(const DeltaConfig& config) {
    s_config = config;
}

DeltaConfig getDeltaConfig() {
    return s_config;
}
