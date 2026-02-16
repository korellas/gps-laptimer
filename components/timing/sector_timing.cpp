/**
 * @file sector_timing.cpp
 * @brief Sector timing management implementation
 * @version 1.0
 */

#include "sector_timing.h"

#include <algorithm>
#include <climits>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SectorTiming";

// ============================================================
// STATIC STATE
// ============================================================

static ReferenceSectorTimes s_refTimes;
static CurrentSectorTiming s_current;
static DeltaDisplayMode s_displayMode = DeltaDisplayMode::LAP_TOTAL;
static unsigned long s_bestSectorTimes[MAX_SECTORS_PER_LAYOUT];
static bool s_initialized = false;

// ============================================================
// INITIALIZATION
// ============================================================

void initSectorTiming() {
    s_refTimes.clear();
    s_current.clear();
    s_displayMode = DeltaDisplayMode::LAP_TOTAL;
    
    // Initialize best times to max
    for (int i = 0; i < MAX_SECTORS_PER_LAYOUT; i++) {
        s_bestSectorTimes[i] = UINT32_MAX;
    }
    
    s_initialized = true;
    printf("[SectorTiming] Initialized\n");
}

void resetSectorTiming() {
    s_current.clear();
    
    // Keep reference times and best times - they persist across laps
    
    // Start in sector 0 if we have sectors
    if (s_refTimes.sectorCount > 0) {
        s_current.totalSectors = s_refTimes.sectorCount;
        s_current.currentSector = 0;
        s_current.sectorEntryTimes[0] = 0;  // Sector 0 starts at lap start
    }
}

void setSectorCount(int count) {
    if (count < 0) count = 0;
    if (count > MAX_SECTORS_PER_LAYOUT) count = MAX_SECTORS_PER_LAYOUT;
    
    s_refTimes.sectorCount = count;
    s_current.totalSectors = count;
    
    printf("[SectorTiming] Sector count set to %d\n", count);
}

// ============================================================
// REFERENCE TIMES
// ============================================================

bool setReferenceSectorTimes(const unsigned long* times, int count) {
    if (times == nullptr || count <= 0) {
        return false;
    }
    
    if (count > MAX_SECTORS_PER_LAYOUT) {
        count = MAX_SECTORS_PER_LAYOUT;
    }
    
    s_refTimes.clear();
    s_refTimes.sectorCount = count;
    
    unsigned long cumulative = 0;
    for (int i = 0; i < count; i++) {
        s_refTimes.sectorTimes[i] = times[i];
        cumulative += times[i];
        s_refTimes.cumulativeTimes[i] = cumulative;
    }
    
    s_current.totalSectors = count;
    
    printf("[SectorTiming] Reference times set for %d sectors\n", count);
    
    return true;
}

bool setReferenceSectorTime(int sectorIndex, unsigned long timeMs) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return false;
    }
    
    s_refTimes.sectorTimes[sectorIndex] = timeMs;
    
    // Recalculate cumulative times
    unsigned long cumulative = 0;
    for (int i = 0; i <= sectorIndex; i++) {
        cumulative += s_refTimes.sectorTimes[i];
        s_refTimes.cumulativeTimes[i] = cumulative;
    }
    
    // Update count if needed
    if (sectorIndex >= s_refTimes.sectorCount) {
        s_refTimes.sectorCount = sectorIndex + 1;
        s_current.totalSectors = s_refTimes.sectorCount;
    }
    
    return true;
}

bool hasReferenceSectorTimes() {
    return s_refTimes.sectorCount > 0 && s_refTimes.sectorTimes[0] > 0;
}

unsigned long getReferenceSectorTime(int sectorIndex) {
    if (sectorIndex < 0 || sectorIndex >= s_refTimes.sectorCount) {
        return 0;
    }
    return s_refTimes.sectorTimes[sectorIndex];
}

unsigned long getReferenceCumulativeTime(int sectorIndex) {
    if (sectorIndex < 0 || sectorIndex >= s_refTimes.sectorCount) {
        return 0;
    }
    return s_refTimes.cumulativeTimes[sectorIndex];
}

const ReferenceSectorTimes& getReferenceSectorTimes() {
    return s_refTimes;
}

// ============================================================
// CURRENT LAP TIMING
// ============================================================

void onSectorEntry(int sectorIndex, unsigned long lapTimeMs) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return;
    }
    
    s_current.currentSector = sectorIndex;
    s_current.sectorEntryTimes[sectorIndex] = lapTimeMs;

    ESP_LOGD(TAG, "Entered S%d at %.3fs", sectorIndex + 1, lapTimeMs / 1000.0f);
}

void onSectorComplete(int sectorIndex, unsigned long lapTimeMs, float totalDeltaSeconds) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return;
    }

    // Calculate sector time
    unsigned long entryTime = s_current.sectorEntryTimes[sectorIndex];
    unsigned long sectorTime = lapTimeMs - entryTime;

    s_current.sectorTimes[sectorIndex] = sectorTime;
    s_current.sectorCompleted[sectorIndex] = true;
    s_current.completedCount++;

    // Record cumulative delta at sector exit
    s_current.cumulativeDeltaAtExit[sectorIndex] = totalDeltaSeconds;

    // Per-sector delta = total delta now - total delta at previous sector exit
    float prevCumulativeDelta = (sectorIndex > 0) ? s_current.cumulativeDeltaAtExit[sectorIndex - 1] : 0.0f;
    float sectorDelta = totalDeltaSeconds - prevCumulativeDelta;
    s_current.sectorDeltas[sectorIndex] = sectorDelta;

    // Update last completed sector info
    s_current.lastCompletedSector = sectorIndex;
    s_current.lastSectorTime = sectorTime;
    s_current.lastSectorDelta = sectorDelta;
    s_current.lastSectorCompletedAt = (unsigned long)(esp_timer_get_time() / 1000ULL);

    // Check for personal best
    if (isSectorBest(sectorIndex, sectorTime)) {
        updateBestSectorTime(sectorIndex, sectorTime);
        ESP_LOGD(TAG, "S%d BEST: %.3fs", sectorIndex + 1, sectorTime / 1000.0f);
    }

    ESP_LOGD(TAG, "S%d complete: %.3fs (sectorDelta: %+.3fs, cumDelta: %+.3fs)",
             sectorIndex + 1, sectorTime / 1000.0f, sectorDelta, totalDeltaSeconds);
}

const CurrentSectorTiming& getCurrentSectorTiming() {
    return s_current;
}

CurrentSectorTiming& getCurrentSectorTimingMutable() {
    return s_current;
}

// ============================================================
// SECTOR DELTA QUERIES
// ============================================================

float getSectorDelta(int sectorIndex) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return 0.0f;
    }
    
    if (!s_current.sectorCompleted[sectorIndex]) {
        return 0.0f;
    }
    
    return s_current.sectorDeltas[sectorIndex];
}

float getLastCompletedSectorDelta() {
    if (s_current.lastCompletedSector < 0) {
        return 0.0f;
    }
    return s_current.lastSectorDelta;
}

unsigned long getCompletedSectorTime(int sectorIndex) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return 0;
    }
    
    if (!s_current.sectorCompleted[sectorIndex]) {
        return 0;
    }
    
    return s_current.sectorTimes[sectorIndex];
}

unsigned long getCumulativeAtSector(int sectorIndex) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return 0;
    }
    
    if (!s_current.sectorCompleted[sectorIndex]) {
        return 0;
    }
    
    // 섹터 진입시간 + 해당 섹터 소요시간 = 누적 시간 (O(1), 루프 불필요)
    return s_current.sectorEntryTimes[sectorIndex] + s_current.sectorTimes[sectorIndex];
}

bool isSectorCompleted(int sectorIndex) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return false;
    }
    return s_current.sectorCompleted[sectorIndex];
}

// ============================================================
// CURRENT SECTOR PROGRESS
// ============================================================

int getCurrentSector() {
    return s_current.currentSector;
}

unsigned long getCurrentSectorElapsed(unsigned long currentLapTimeMs) {
    int sector = s_current.currentSector;
    if (sector < 0 || sector >= MAX_SECTORS_PER_LAYOUT) {
        return 0;
    }
    
    unsigned long entryTime = s_current.sectorEntryTimes[sector];
    if (currentLapTimeMs < entryTime) {
        return 0;
    }
    
    return currentLapTimeMs - entryTime;
}

float getCurrentSectorInProgressDelta(unsigned long currentLapTimeMs) {
    int sector = s_current.currentSector;
    if (sector < 0 || sector >= s_refTimes.sectorCount) {
        return 0.0f;
    }
    
    // Get elapsed time in current sector
    unsigned long elapsed = getCurrentSectorElapsed(currentLapTimeMs);
    
    // Get reference sector time
    unsigned long refSectorTime = s_refTimes.sectorTimes[sector];
    if (refSectorTime == 0) {
        return 0.0f;
    }
    
    // Estimate progress through sector (0-1)
    float progress = (float)elapsed / (float)refSectorTime;
    progress = std::clamp(progress, 0.0f, SECTOR_PROGRESS_MAX);
    
    // Calculate expected time at this progress
    float expectedElapsed = progress * refSectorTime;
    
    // Delta = actual - expected
    return (float)(elapsed - expectedElapsed) / 1000.0f;
}

// ============================================================
// DISPLAY MODE
// ============================================================

DeltaDisplayMode getSectorDeltaDisplayMode() {
    return s_displayMode;
}

void setSectorDeltaDisplayMode(DeltaDisplayMode mode) {
    s_displayMode = mode;
}

float getDisplayDelta(float lapDelta, unsigned long currentLapTimeMs) {
    switch (s_displayMode) {
        case DeltaDisplayMode::LAP_TOTAL:
            return lapDelta;
            
        case DeltaDisplayMode::CURRENT_SECTOR:
            return getCurrentSectorInProgressDelta(currentLapTimeMs);
            
        case DeltaDisplayMode::LAST_SECTOR:
            // Show last sector delta for a few seconds after completion
            if (s_current.lastCompletedSector >= 0) {
                unsigned long elapsed = (unsigned long)(esp_timer_get_time() / 1000ULL) -
                                        s_current.lastSectorCompletedAt;
                if (elapsed < SECTOR_DELTA_DISPLAY_MS) {
                    return s_current.lastSectorDelta;
                }
            }
            // Fall back to lap delta
            return lapDelta;
            
        default:
            return lapDelta;
    }
}

// ============================================================
// BEST SECTOR TRACKING
// ============================================================

bool isSectorBest(int sectorIndex, unsigned long timeMs) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return false;
    }
    
    if (timeMs == 0) {
        return false;
    }
    
    return timeMs < s_bestSectorTimes[sectorIndex];
}

void updateBestSectorTime(int sectorIndex, unsigned long timeMs) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return;
    }
    
    if (timeMs > 0 && timeMs < s_bestSectorTimes[sectorIndex]) {
        s_bestSectorTimes[sectorIndex] = timeMs;
    }
}

unsigned long getBestSectorTime(int sectorIndex) {
    if (sectorIndex < 0 || sectorIndex >= MAX_SECTORS_PER_LAYOUT) {
        return UINT32_MAX;
    }
    return s_bestSectorTimes[sectorIndex];
}

// ============================================================
// SECTOR DISTANCE MANAGEMENT
// ============================================================

// Internal: sector boundary distances (between sector N and N+1)
static float s_sectorBoundaryDistances[MAX_SECTORS_PER_LAYOUT];
static float s_totalTrackDistance = 0.0f;
static int s_numBoundaries = 0;

void updateSectorDistancesFromReference(const GPSPoint* refPoints,
                                         const float* cumDist,
                                         int numPoints,
                                         const SectorBoundaryPoint* boundaries,
                                         int numBoundaries) {
    if (refPoints == nullptr || cumDist == nullptr || numPoints < 2 ||
        boundaries == nullptr || numBoundaries <= 0) {
        return;
    }

    s_totalTrackDistance = cumDist[numPoints - 1];
    s_numBoundaries = numBoundaries;

    for (int b = 0; b < numBoundaries; b++) {
        float minDist = 1e9f;
        int closestIdx = 0;

        for (int i = 0; i < numPoints; i++) {
            float dx = (float)(refPoints[i].lat - boundaries[b].lat);
            float dy = (float)(refPoints[i].lng - boundaries[b].lng);
            // Fast squared-distance in degree space (sufficient for finding closest)
            float distSq = dx * dx + dy * dy;
            if (distSq < minDist) {
                minDist = distSq;
                closestIdx = i;
            }
        }

        s_sectorBoundaryDistances[b] = cumDist[closestIdx];
        printf("[SectorTiming] Boundary S%d→S%d at ref point %d, distance=%.1fm\n",
               b + 1, b + 2, closestIdx, s_sectorBoundaryDistances[b]);
    }

    // Set sector count: boundaries + 1
    int sectorCount = numBoundaries + 1;
    setSectorCount(sectorCount);

    printf("[SectorTiming] %d sectors, track=%.1fm, bounds=[",
           sectorCount, s_totalTrackDistance);
    for (int i = 0; i < numBoundaries; i++) {
        printf("%.1f%s", s_sectorBoundaryDistances[i], i < numBoundaries - 1 ? ", " : "");
    }
    printf("]\n");
}

int checkSectorTransitionByDistance(float trackDistanceM) {
    if (s_current.totalSectors <= 1 || s_numBoundaries == 0) {
        return -1;
    }

    int currentSector = s_current.currentSector;
    if (currentSector < 0) {
        return -1;
    }

    // Check if we've crossed into the next sector
    // Boundary index = currentSector (boundary between sector N and N+1)
    if (currentSector < s_numBoundaries) {
        float boundaryDist = s_sectorBoundaryDistances[currentSector];
        if (trackDistanceM >= boundaryDist) {
            return currentSector;  // Current sector just completed
        }
    }

    return -1;
}

int getSectorBoundaryDistances(float* outDistances, int maxCount) {
    int count = (s_numBoundaries < maxCount) ? s_numBoundaries : maxCount;
    for (int i = 0; i < count; i++) {
        outDistances[i] = s_sectorBoundaryDistances[i];
    }
    return count;
}