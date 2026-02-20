/**
 * @file simulation.cpp
 * @brief Simulation mode implementation for GPS Lap Timer
 * @version 1.0
 * 
 * Handles playback of recorded lap data for development and testing.
 * Simulates GPS data at realistic timing intervals.
 */

#include "simulation.h"

#include <algorithm>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Simulation";
#include "config.h"
#include "types.h"
#include "../geo/geo_utils.h"
#include "waveshare_display.h"
#include "finish_line.h"
#include "protocol.hpp"
#include "../timing/delta_calculator.h"
#include "../timing/sector_timing.h"
#include "../track/builtin_tracks.h"
#include "../track/track_manager.h"
#include "../../examples/everland_track_data.h"
#include "../../examples/everland_reference_5283.h"

// 에버랜드 섹터 경계 좌표 (SSOT: builtin_tracks.h → everland::SECTOR_BOUNDARIES)
static const SectorBoundaryPoint s_everlandSectorBoundaries[] = {
    {37.2974004, 127.2175125},   // S1→S2
    {37.2961782, 127.2138076},   // S2→S3
};
static constexpr int EVERLAND_SECTOR_BOUNDARY_COUNT = sizeof(s_everlandSectorBoundaries) / sizeof(s_everlandSectorBoundaries[0]);

// ============================================================
// EXTERNAL REFERENCES
// ============================================================

// Function prototypes from main.cpp
extern void updateDisplayData(const GPSPoint& point, const DeltaResult& delta,
                              unsigned long lapTimeMs, float refTimeSec);
extern void updateTop3Laps(uint16_t lapNumber, uint32_t lapTimeMs);

// ============================================================
// MODULE STATE
// ============================================================

// Simulation state
static SimulationState simState;

// Current simulation lap data (points being played back) - 고정 배열로 힙 단편화 방지
static GPSPoint simLapPoints[MAX_REFERENCE_POINTS];
static int simLapPointCount = 0;

// Track which data lap index is our current best
static int bestLapDataIdx = -1;

// ============================================================
// HELPER FUNCTIONS
// ============================================================

/**
 * @brief Find the best (fastest) sample lap index from track data
 */
static int findBestSampleLapIndex() {
    int bestIdx = 0;
    uint32_t bestTime = UINT32_MAX;
    
    for (int i = 0; i < NUM_LAPS; i++) {
        if (lap_boundaries[i].lapTimeMs < bestTime) {
            bestTime = lap_boundaries[i].lapTimeMs;
            bestIdx = i;
        }
    }
    return bestIdx;
}

/**
 * @brief Check if finish line was crossed between two points
 */
static bool checkSimFinishLineCrossing(double p1Lat, double p1Lng, double p2Lat, double p2Lng) {
    return segmentsIntersect(
        p1Lat, p1Lng, p2Lat, p2Lng,
        FINISH_LINE_A_LAT, FINISH_LINE_A_LNG,
        FINISH_LINE_B_LAT, FINISH_LINE_B_LNG
    );
}

// ============================================================
// REFERENCE LAP LOADING (이전 lap_manager에서 이전)
// ============================================================

static bool loadRefLapFromTrackData(int lapIdx) {
    const LapBoundary* boundaries = getEverlandLapBoundaries();
    const TrackPoint* points = getEverlandTrackPoints();
    int lapCount = getEverlandLapCount();

    if (lapIdx < 0 || lapIdx >= lapCount) {
        ESP_LOGW(TAG, "Invalid lap index: %d", lapIdx);
        return false;
    }

    const LapBoundary& bounds = boundaries[lapIdx];

    LapData lap;
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

        if (i % 100 == 0) {
            taskYIELD();
        }
    }

    if (lap.points.empty()) return false;

    lap.startTimeMs = 0;
    lap.totalTimeMs = bounds.lapTimeMs;
    lap.maxSpeedKmh = maxSpeed;
    lap.avgSpeedKmh = totalSpeed / lap.points.size();

    if (!setReferenceLap(lap)) return false;

    gApp.bestLapTimeMs = bounds.lapTimeMs;
    gApp.hasValidReferenceLap = true;

    ESP_LOGI(TAG, "Loaded track data lap %d: %d pts, %.2fs",
             lapIdx, (int)lap.points.size(), lap.totalTimeMs / 1000.0f);
    return true;
}

static bool loadReferenceLapFromPriorSession() {
    const auto* refPoints = getEverlandReferencePoints();
    int refCount = getEverlandReferencePointCount();

    if (refPoints == nullptr || refCount <= 0) return false;

    LapData lap;
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

    if (lap.points.empty()) return false;

    lap.startTimeMs = 0;
    lap.totalTimeMs = lap.points.back().lapTimeMs;
    lap.maxSpeedKmh = maxSpeed;
    lap.avgSpeedKmh = totalSpeed / lap.points.size();

    if (!setReferenceLap(lap)) return false;

    gApp.bestLapTimeMs = lap.totalTimeMs;
    gApp.hasValidReferenceLap = true;

    ESP_LOGI(TAG, "Loaded prior session reference: %d pts, %.2fs",
             (int)lap.points.size(), lap.totalTimeMs / 1000.0f);
    return true;
}

// ============================================================
// PUBLIC API IMPLEMENTATION
// ============================================================

bool loadSimulationLap(int lapIdx) {
    if (lapIdx < 0 || lapIdx >= NUM_LAPS) {
        return false;
    }

    const LapBoundary& bounds = lap_boundaries[lapIdx];
    simLapPointCount = 0;

    for (int i = bounds.startIndex; i <= bounds.endIndex && simLapPointCount < MAX_REFERENCE_POINTS; i++) {
        const TrackPoint& tp = track_points[i];

        GPSPoint& point = simLapPoints[simLapPointCount];
        point.lat = tp.lat / 1e7;
        point.lng = tp.lon / 1e7;
        point.lapTimeMs = tp.timeMs;
        point.speedKmh = tp.speedX10 / 10.0f;
        point.headingDeg = 0.0f;
        point.gpsTimeMs = tp.timeMs;

        simLapPointCount++;
    }

    return simLapPointCount > 0;
}

void initializeSimulation() {
    simState.currentLapIdx = 0;  // Start with first lap in data
    simState.reset();
    bestLapDataIdx = -1;
    
    // Activate track for sector detection
    if (setActiveTrackById("everland", "full")) {
        printf("[Sim] Track activated for sector detection\n");
    }
    
    // Initialize sector timing (distance-based)
    initSectorTiming();

    // Try to load prior session reference first (via lap_manager)
    if (loadReferenceLapFromPriorSession()) {
        bestLapDataIdx = -1;
        lframe.bestLapNumber = 0;  // Prior-session best (not this session)
    } else if (!gApp.hasValidReferenceLap) {
        // Fallback: use best lap from track data (via lap_manager)
        int refIdx = findBestSampleLapIndex();
        if (loadRefLapFromTrackData(refIdx)) {
            bestLapDataIdx = refIdx;
            lframe.bestLapNumber = 0;
        }
    }

    if (!gApp.hasValidReferenceLap) {
        gApp.bestLapTimeMs = UINT32_MAX;
    }

    // Calculate sector boundary distances from reference lap
    if (gApp.hasValidReferenceLap && !gApp.referenceLap.points.empty()) {
        updateSectorDistancesFromReference(
            gApp.referenceLap.points.data(),
            gApp.referenceLap.cumulativeDistances.data(),
            (int)gApp.referenceLap.points.size(),
            s_everlandSectorBoundaries, EVERLAND_SECTOR_BOUNDARY_COUNT);
        resetSectorTiming();
    }

    // Reset completed lap display state
    lframe.lastCompletedLapMs = 0;
    lframe.lapCompleteDisplayEndMs = 0;
    if (!gApp.hasValidReferenceLap) {
        lframe.bestLapNumber = 0;
    }

    // Load first lap for playback
    loadSimulationLap(simState.currentLapIdx);

    printf("\n=== SIMULATION START ===\n");
    printf("Track: Everland\n");
    printf("Available laps: %d\n", NUM_LAPS);
    printf("%s\n\n",
                  gApp.hasValidReferenceLap ? "Using prior-session reference lap" : "First lap (no reference) - recording only");
}

void onSimLapComplete(unsigned long lapTimeMs) {
    uint16_t completedLapNumber = gApp.currentLapNumber;
    printf("\n=== LAP %u COMPLETE ===\n", completedLapNumber);
    printf("  Lap time: %.3fs\n", lapTimeMs / 1000.0f);

    if (gApp.hasValidReferenceLap) {
        printf("  Delta: %+.3fs vs best (%.3fs)\n",
            gApp.currentDelta.deltaSeconds, gApp.bestLapTimeMs / 1000.0f);
    } else {
        printf("  (First lap - no comparison)\n");
    }

    // Update top 3 laps for display
    updateTop3Laps(completedLapNumber, lapTimeMs);

    // Set completed lap display (show for LAP_COMPLETE_DISPLAY_MS)
    lframe.lastCompletedLapMs = lapTimeMs;
    lframe.lapCompleteDisplayEndMs = (unsigned long)(esp_timer_get_time() / 1000ULL) + LAP_COMPLETE_DISPLAY_MS;

    // Check if this lap is the new best
    bool isNewBest = lapTimeMs < gApp.bestLapTimeMs;

    if (isNewBest) {
        printf("  *** NEW BEST LAP! ***\n");
        gApp.bestLapTimeMs = lapTimeMs;
        bestLapDataIdx = simState.currentLapIdx;

        // Update best lap number (use current lap number before increment)
        lframe.bestLapNumber = gApp.currentLapNumber;

        // Load the just-completed lap as the new reference (via lap_manager)
        loadRefLapFromTrackData(bestLapDataIdx);
        gApp.bestLapTimeMs = lapTimeMs;  // Preserve actual measured time
        printf("  Reference updated to lap data %d (%.3fs)\n",
            bestLapDataIdx, gApp.bestLapTimeMs / 1000.0f);

        // Recalculate sector distances for new reference
        if (!gApp.referenceLap.points.empty()) {
            updateSectorDistancesFromReference(
                gApp.referenceLap.points.data(),
                gApp.referenceLap.cumulativeDistances.data(),
                (int)gApp.referenceLap.points.size(),
                s_everlandSectorBoundaries, EVERLAND_SECTOR_BOUNDARY_COUNT);
        }
    }

    // After first lap completes, we now have a reference for future laps
    if (!gApp.hasValidReferenceLap) {
        gApp.hasValidReferenceLap = true;
    }

    // Move to next lap
    gApp.currentLapNumber++;
    simState.startNextLap();

    // Load next lap data for playback
    loadSimulationLap(simState.currentLapIdx);

    // Clear points
    gApp.currentPoint.clear();
    gApp.previousPoint.clear();
    gApp.currentDelta.clear();
    gApp.lastValidSegmentIndex = -1;

    resetDeltaHistory();
    resetCrossingState();
    resetSectorTiming();

    printf("  Starting lap %u (playing data lap %d)\n\n",
        gApp.currentLapNumber, simState.currentLapIdx);
}

void resetSimulation() {
    initializeSimulation();
    gApp.currentLapNumber = 1;
    resetDeltaHistory();
    resetCrossingState();
    resetSectorTiming();

    gApp.currentPoint.clear();
    gApp.previousPoint.clear();
    gApp.currentDelta.clear();
    gApp.lastValidSegmentIndex = -1;

    printf("Simulation reset - Lap %u starting\n", gApp.currentLapNumber);
}

void processSimulation() {
    unsigned long now = (unsigned long)(esp_timer_get_time() / 1000ULL);

    if (now - simState.lastUpdateMs < SIM_UPDATE_INTERVAL_MS) {
        return;
    }
    simState.lastUpdateMs = now;

    // LAP TIME = real wall clock time (always ticking, never stops)
    unsigned long lapTimeMs = now - simState.startMillis;

    int numPoints = simLapPointCount;

    // Track if we got new GPS data this frame
    bool gotNewGpsData = false;

    // Find GPS points that should have arrived by now
    while (simState.currentPointIdx < numPoints) {
        const GPSPoint& point = simLapPoints[simState.currentPointIdx];
        unsigned long gpsTimeMs = point.lapTimeMs;  // GPS data timestamp

        if (gpsTimeMs <= lapTimeMs) {
            // New GPS data arrived
            gApp.previousPoint = gApp.currentPoint;
            gApp.currentPoint = point;
            // Keep original GPS data timestamp for accurate delta calculation
            gotNewGpsData = true;

            // Calculate delta using original GPS timestamps (not wall clock)
            if (gApp.hasValidReferenceLap && !simState.isDeltaSuppressed()) {
                gApp.currentDelta = calculateDelta(gApp.currentPoint, gApp.referenceLap, &gApp.previousPoint);
            } else {
                gApp.currentDelta.clear();
            }

            // Sector transition detection (distance-based)
            if (gApp.hasValidReferenceLap && !simState.isDeltaSuppressed() &&
                gApp.currentDelta.trackDistanceM >= 0) {
                int completedSector = checkSectorTransitionByDistance(gApp.currentDelta.trackDistanceM);
                if (completedSector >= 0) {
                    onSectorComplete(completedSector, gpsTimeMs, gApp.currentDelta.deltaSeconds);
                    int nextSector = completedSector + 1;
                    const CurrentSectorTiming& st = getCurrentSectorTiming();
                    if (nextSector < st.totalSectors) {
                        onSectorEntry(nextSector, gpsTimeMs);
                    }
                }
            }

            // Override with wall clock AFTER delta calc (for display/interpolation)
            gApp.currentPoint.lapTimeMs = lapTimeMs;

            simState.currentPointIdx++;
            simState.initialized = true;

            // Check finish line crossing (need previous point and min lap time)
            if (gApp.previousPoint.isValid() && lapTimeMs > MIN_LAP_TIME_MS) {
                if (checkSimFinishLineCrossing(gApp.previousPoint.lat, gApp.previousPoint.lng,
                                                point.lat, point.lng)) {
                    onSimLapComplete(lapTimeMs);
                    return;
                }
            }
        } else {
            // No more GPS data available yet
            break;
        }
    }

    // GPS data gap handling: time keeps ticking, interpolate speed only
    // Delta is kept as-is (can't accurately compute without GPS position)
    if (!gotNewGpsData && simState.initialized) {
        // No new GPS data, but time marches on
        if (simState.currentPointIdx < numPoints) {
            const GPSPoint& nextPoint = simLapPoints[simState.currentPointIdx];
            unsigned long nextGpsTime = nextPoint.lapTimeMs;
            unsigned long prevGpsTime = gApp.currentPoint.lapTimeMs;

            if (nextGpsTime > prevGpsTime) {
                float t = (float)(lapTimeMs - prevGpsTime) / (nextGpsTime - prevGpsTime);
                t = std::clamp(t, 0.0f, 1.0f);

                // Interpolate speed linearly
                gApp.currentPoint.speedKmh = gApp.currentPoint.speedKmh + t * (nextPoint.speedKmh - gApp.currentPoint.speedKmh);
            }
        }
        // Delta: keep last calculated value - interpolating causes accumulation errors
    }

    // Always update display with real lap time
    updateDisplayData(gApp.currentPoint, gApp.currentDelta, lapTimeMs, gApp.currentDelta.refTimeSec);

    static unsigned long lastDebugMs = 0;
    if (now - lastDebugMs >= DEBUG_INTERVAL_MS) {
        lastDebugMs = now;
        ESP_LOGD(TAG, "L%d[%d/%d] time=%.1fs delta=%+.2fs spd=%.0fkm/h %s%s",
                 gApp.currentLapNumber,
                 simState.currentPointIdx, numPoints,
                 lapTimeMs / 1000.0f,
                 gApp.currentDelta.deltaSeconds,
                 gApp.currentPoint.speedKmh,
                 gApp.hasValidReferenceLap ? "" : "(no ref)",
                 gotNewGpsData ? "" : " [interp]");
    }

    // Check if lap data ended
    if (simState.currentPointIdx >= numPoints) {
        printf("=== Lap data ended ===\n");
        onSimLapComplete(lapTimeMs);
    }
}

// ============================================================
// STATE ACCESSORS
// ============================================================

const SimulationState& getSimulationState() {
    return simState;
}

SimulationState& getSimulationStateMutable() {
    return simState;
}

int getCurrentSimLapDataIndex() {
    return simState.currentLapIdx;
}

int getBestSimLapDataIndex() {
    return bestLapDataIdx;
}

const GPSPoint* getSimLapPoints() {
    return simLapPoints;
}

int getSimLapPointCount() {
    return simLapPointCount;
}