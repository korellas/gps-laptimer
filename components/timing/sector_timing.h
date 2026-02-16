/**
 * @file sector_timing.h
 * @brief Sector timing management for split time tracking
 * @version 1.0
 * 
 * Manages sector timing state and delta calculations for split timing.
 * Works with the sector detection module to track times through
 * each sector of the track.
 */

#ifndef SECTOR_TIMING_H
#define SECTOR_TIMING_H

#include "types.h"
#include "config.h"

// ============================================================
// REFERENCE SECTOR TIMES
// ============================================================

/**
 * @brief Reference times for each sector
 */
struct ReferenceSectorTimes {
    unsigned long sectorTimes[MAX_SECTORS_PER_LAYOUT];  // Time for each sector (ms)
    unsigned long cumulativeTimes[MAX_SECTORS_PER_LAYOUT];  // Cumulative at each sector end
    int sectorCount;
    
    void clear() {
        for (int i = 0; i < MAX_SECTORS_PER_LAYOUT; i++) {
            sectorTimes[i] = 0;
            cumulativeTimes[i] = 0;
        }
        sectorCount = 0;
    }
};

/**
 * @brief Current lap sector timing state
 */
struct CurrentSectorTiming {
    // Current sector times
    unsigned long sectorTimes[MAX_SECTORS_PER_LAYOUT];
    unsigned long sectorEntryTimes[MAX_SECTORS_PER_LAYOUT];
    bool sectorCompleted[MAX_SECTORS_PER_LAYOUT];
    
    // Sector deltas (cumulative approach: all sector deltas sum to total lap delta)
    float sectorDeltas[MAX_SECTORS_PER_LAYOUT];  // Per-sector delta (seconds)
    float cumulativeDeltaAtExit[MAX_SECTORS_PER_LAYOUT];  // Total lap delta at sector exit

    // Current state
    int currentSector;           // Current sector index (-1 = not started)
    int completedCount;          // Number of completed sectors
    int totalSectors;            // Total sectors in layout
    
    // Last completed sector info (for display)
    int lastCompletedSector;
    unsigned long lastSectorTime;
    float lastSectorDelta;
    unsigned long lastSectorCompletedAt;  // millis() when completed
    
    void clear() {
        for (int i = 0; i < MAX_SECTORS_PER_LAYOUT; i++) {
            sectorTimes[i] = 0;
            sectorEntryTimes[i] = 0;
            sectorCompleted[i] = false;
            sectorDeltas[i] = 0.0f;
            cumulativeDeltaAtExit[i] = 0.0f;
        }
        currentSector = -1;
        completedCount = 0;
        totalSectors = 0;
        lastCompletedSector = -1;
        lastSectorTime = 0;
        lastSectorDelta = 0.0f;
        lastSectorCompletedAt = 0;
    }
};

// ============================================================
// INITIALIZATION
// ============================================================

/**
 * @brief Initialize sector timing
 */
void initSectorTiming();

/**
 * @brief Reset sector timing for new lap
 * 
 * Clears current sector times and state.
 */
void resetSectorTiming();

/**
 * @brief Configure sector count
 * 
 * @param count Number of sectors
 */
void setSectorCount(int count);

// ============================================================
// REFERENCE TIMES
// ============================================================

/**
 * @brief Set reference sector times
 * 
 * @param times Reference sector times array
 * @param count Number of sectors
 * @return true if set successfully
 */
bool setReferenceSectorTimes(const unsigned long* times, int count);

/**
 * @brief Set reference sector time for single sector
 * 
 * @param sectorIndex Sector index
 * @param timeMs Sector time in milliseconds
 * @return true if set successfully
 */
bool setReferenceSectorTime(int sectorIndex, unsigned long timeMs);

/**
 * @brief Check if reference sector times are available
 * 
 * @return true if reference times are set
 */
bool hasReferenceSectorTimes();

/**
 * @brief Get reference time for a sector
 * 
 * @param sectorIndex Sector index
 * @return Reference time in ms, or 0 if not available
 */
unsigned long getReferenceSectorTime(int sectorIndex);

/**
 * @brief Get reference cumulative time at sector end
 * 
 * @param sectorIndex Sector index
 * @return Cumulative time in ms, or 0 if not available
 */
unsigned long getReferenceCumulativeTime(int sectorIndex);

/**
 * @brief Get reference sector times struct
 * 
 * @return Const reference to reference times
 */
const ReferenceSectorTimes& getReferenceSectorTimes();

// ============================================================
// CURRENT LAP TIMING
// ============================================================

/**
 * @brief Record sector entry
 * 
 * Called when car enters a new sector.
 * 
 * @param sectorIndex Sector being entered
 * @param lapTimeMs Current lap time
 */
void onSectorEntry(int sectorIndex, unsigned long lapTimeMs);

/**
 * @brief Record sector completion
 *
 * Called when car exits a sector. Records cumulative delta at exit.
 * Per-sector delta = cumulative delta at this exit - cumulative delta at previous exit.
 *
 * @param sectorIndex Sector that was completed
 * @param lapTimeMs Current lap time
 * @param totalDeltaSeconds Total lap delta at this moment (from calculateDelta)
 */
void onSectorComplete(int sectorIndex, unsigned long lapTimeMs, float totalDeltaSeconds);

/**
 * @brief Get current sector timing state
 * 
 * @return Const reference to current timing state
 */
const CurrentSectorTiming& getCurrentSectorTiming();

/**
 * @brief Get current sector timing state (mutable)
 * 
 * @return Reference to current timing state
 */
CurrentSectorTiming& getCurrentSectorTimingMutable();

// ============================================================
// SECTOR DELTA QUERIES
// ============================================================

/**
 * @brief Get delta for a completed sector
 * 
 * @param sectorIndex Sector index
 * @return Delta in seconds (+slower, -faster), or 0 if not available
 */
float getSectorDelta(int sectorIndex);

/**
 * @brief Get delta for the most recently completed sector
 * 
 * @return Delta in seconds
 */
float getLastCompletedSectorDelta();

/**
 * @brief Get time for a completed sector
 * 
 * @param sectorIndex Sector index
 * @return Sector time in ms, or 0 if not completed
 */
unsigned long getCompletedSectorTime(int sectorIndex);

/**
 * @brief Get cumulative time at end of a sector
 * 
 * @param sectorIndex Sector index
 * @return Cumulative time in ms, or 0 if sector not completed
 */
unsigned long getCumulativeAtSector(int sectorIndex);

/**
 * @brief Check if a sector is completed
 * 
 * @param sectorIndex Sector index
 * @return true if sector has been completed this lap
 */
bool isSectorCompleted(int sectorIndex);

// ============================================================
// CURRENT SECTOR PROGRESS
// ============================================================

/**
 * @brief Get current sector index
 * 
 * @return Current sector (0-based), or -1 if not started
 */
int getCurrentSector();

/**
 * @brief Get time spent in current sector so far
 * 
 * @param currentLapTimeMs Current total lap time
 * @return Time in current sector in ms
 */
unsigned long getCurrentSectorElapsed(unsigned long currentLapTimeMs);

/**
 * @brief Get in-progress delta for current sector
 * 
 * Estimates delta based on elapsed time in current sector
 * compared to reference.
 * 
 * @param currentLapTimeMs Current total lap time
 * @return Estimated delta in seconds
 */
float getCurrentSectorInProgressDelta(unsigned long currentLapTimeMs);

// ============================================================
// DISPLAY MODE
// ============================================================

/**
 * @brief Get current delta display mode
 * 
 * @return Current display mode
 */
DeltaDisplayMode getSectorDeltaDisplayMode();

/**
 * @brief Set delta display mode
 * 
 * @param mode Display mode to use
 */
void setSectorDeltaDisplayMode(DeltaDisplayMode mode);

/**
 * @brief Get the delta value to display based on current mode
 * 
 * @param lapDelta Full lap delta (for LAP_TOTAL mode)
 * @param currentLapTimeMs Current lap time (for sector calculations)
 * @return Delta value to display in seconds
 */
float getDisplayDelta(float lapDelta, unsigned long currentLapTimeMs);

// ============================================================
// BEST SECTOR TRACKING
// ============================================================

/**
 * @brief Check if sector time is a personal best
 * 
 * @param sectorIndex Sector index
 * @param timeMs Sector time to check
 * @return true if this is a new best for this sector
 */
bool isSectorBest(int sectorIndex, unsigned long timeMs);

/**
 * @brief Update best sector time
 * 
 * @param sectorIndex Sector index
 * @param timeMs New best time
 */
void updateBestSectorTime(int sectorIndex, unsigned long timeMs);

/**
 * @brief Get best time for a sector
 * 
 * @param sectorIndex Sector index
 * @return Best sector time in ms, or UINT32_MAX if none
 */
unsigned long getBestSectorTime(int sectorIndex);

// ============================================================
// SECTOR DISTANCE MANAGEMENT
// ============================================================

/**
 * @brief Sector boundary coordinate for distance calculation
 */
struct SectorBoundaryPoint {
    double lat;
    double lng;
};

/**
 * @brief Update sector boundary distances from reference lap
 *
 * Finds the closest reference lap point to each sector boundary coordinate
 * and uses its cumulative distance as the sector boundary distance.
 *
 * @param refPoints Reference lap points
 * @param cumDist Cumulative distances array
 * @param numPoints Number of points
 * @param boundaries Sector boundary coordinates
 * @param numBoundaries Number of boundaries
 */
void updateSectorDistancesFromReference(const GPSPoint* refPoints,
                                         const float* cumDist,
                                         int numPoints,
                                         const SectorBoundaryPoint* boundaries,
                                         int numBoundaries);

/**
 * @brief Check if track distance crossed a sector boundary
 *
 * @param trackDistanceM Current track distance
 * @return Index of sector just completed, or -1 if no transition
 */
int checkSectorTransitionByDistance(float trackDistanceM);

/**
 * @brief Get sector boundary distances (for debugging)
 *
 * @param outDistances Output array (size >= totalSectors - 1)
 * @return Number of boundary distances written
 */
int getSectorBoundaryDistances(float* outDistances, int maxCount);

#endif // SECTOR_TIMING_H