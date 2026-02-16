/**
 * @file delta_calculator.h
 * @brief Delta time calculation between current position and reference lap
 * @version 1.0
 * 
 * Calculates the time difference (delta) between the current lap
 * and a reference lap based on track position. Supports both
 * full lap delta and sector-based delta modes.
 */

#ifndef DELTA_CALCULATOR_H
#define DELTA_CALCULATOR_H

#include "types.h"

// ============================================================
// DELTA CALCULATION CONFIGURATION
// ============================================================

/**
 * @brief Configuration for delta calculation behavior
 */
struct DeltaConfig {
    // Maximum distance from track to consider valid (meters)
    float maxProjectionDistanceM = 50.0f;
    
    // Confidence decay factor (higher = faster decay when far from track)
    float confidenceDecayFactor = 3.0f;
    
    // Segment search window size for continuity
    int segmentSearchWindow = 50;
    
    // Backward search allowance (for slight backtracking)
    int segmentSearchBack = 10;
    
    // Minimum confidence to consider delta valid
    float minConfidence = 0.1f;
};

// ============================================================
// INITIALIZATION
// ============================================================

/**
 * @brief Initialize the delta calculator
 * 
 * @param config Configuration parameters (uses defaults if not specified)
 */
void initDeltaCalculator(const DeltaConfig& config = DeltaConfig());

/**
 * @brief Reset delta calculator state
 * 
 * Clears segment tracking and history. Call at lap start.
 */
void resetDeltaCalculator();

// ============================================================
// REFERENCE LAP MANAGEMENT
// ============================================================

/**
 * @brief Set the reference lap for delta calculations
 * 
 * The reference lap is used for comparison. This also pre-calculates
 * cumulative distances needed for track position lookup.
 * 
 * @param lap Reference lap data
 * @return true if reference was set successfully
 */
bool setReferenceLap(const LapData& lap);

/**
 * @brief Check if a valid reference lap is set
 * 
 * @return true if reference lap is available
 */
bool hasReferenceLap();

/**
 * @brief Get the current reference lap
 * 
 * @return Const reference to the reference lap data
 */
const LapData& getReferenceLap();

/**
 * @brief Clear the reference lap
 */
void clearReferenceLap();

/**
 * @brief Get reference lap total time
 * 
 * @return Reference lap time in milliseconds
 */
unsigned long getReferenceLapTimeMs();

// ============================================================
// PRE-CALCULATION
// ============================================================

/**
 * @brief Pre-calculate cumulative distances for a lap
 * 
 * This must be called after loading reference lap data.
 * Populates the cumulativeDistances vector in the LapData.
 * 
 * @param lap Lap data to calculate distances for
 */
void calculateCumulativeDistances(LapData& lap);

// ============================================================
// DELTA CALCULATION
// ============================================================

/**
 * @brief Calculate delta for current position
 * 
 * Main delta calculation function. Finds the closest point on the
 * reference lap and computes time difference.
 * 
 * Algorithm:
 * 1. Find nearest segment on reference lap (with continuity preference)
 * 2. Project current position onto segment
 * 3. Calculate track distance at projected point
 * 4. Interpolate reference time at that track distance
 * 5. Delta = current lap time - reference time
 * 
 * @param current Current GPS position with lap time
 * @param reference Reference lap data for comparison
 * @param previousPoint Optional previous point for speed delta fallback
 * @return Delta result with time difference and metadata
 */
DeltaResult calculateDelta(const GPSPoint& current, const LapData& reference,
                           const GPSPoint* previousPoint = nullptr);

/**
 * @brief Calculate delta at a specific track distance
 * 
 * Useful when track distance is already known (e.g., from sector detection).
 * 
 * @param trackDistanceM Distance along track from start
 * @param currentLapTimeMs Current lap time
 * @return Delta result
 */
DeltaResult calculateDeltaAtDistance(float trackDistanceM, unsigned long currentLapTimeMs);

/**
 * @brief Get reference time at a track distance
 * 
 * Interpolates the reference lap time at a given track distance.
 * 
 * @param trackDistanceM Distance along track from start
 * @return Reference time in milliseconds, or 0 if invalid
 */
unsigned long getReferenceTimeAtDistance(float trackDistanceM);

// ============================================================
// SEGMENT TRACKING
// ============================================================

/**
 * @brief Get the last matched segment index
 * 
 * Useful for debugging and continuity checking.
 * 
 * @return Last valid segment index, or -1 if none
 */
int getLastMatchedSegment();

/**
 * @brief Force segment index (for testing or recovery)
 * 
 * @param segmentIndex Segment index to set
 */
void setLastMatchedSegment(int segmentIndex);

// ============================================================
// TRACK PROGRESS
// ============================================================

/**
 * @brief Get current track progress as percentage
 * 
 * @return Progress 0.0 to 1.0, or -1 if unknown
 */
float getTrackProgress();

/**
 * @brief Get current track distance
 * 
 * @return Distance in meters from track start, or -1 if unknown
 */
float getCurrentTrackDistance();

// ============================================================
// CONFIGURATION
// ============================================================

/**
 * @brief Update delta calculator configuration
 * 
 * @param config New configuration
 */
void setDeltaConfig(const DeltaConfig& config);

/**
 * @brief Get current configuration
 * 
 * @return Current configuration
 */
DeltaConfig getDeltaConfig();

#endif // DELTA_CALCULATOR_H