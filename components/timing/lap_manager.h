/**
 * @file lap_manager.h
 * @brief Lap and reference lap management
 * @version 1.0
 * 
 * Manages lap timing state, reference lap loading/saving,
 * and best lap tracking. Coordinates between GPS data,
 * delta calculation, and storage modules.
 */

#ifndef LAP_MANAGER_H
#define LAP_MANAGER_H

#include "types.h"
#include "config.h"

// ============================================================
// LAP STATE
// ============================================================

/**
 * @brief Current lap state information
 */
struct LapState {
    bool lapInProgress;             // Is a lap currently being timed
    unsigned long lapStartMs;       // When current lap started (millis)
    unsigned long currentLapTimeMs; // Current lap elapsed time

    // Last completed lap
    unsigned long lastLapTimeMs;    // Last completed lap time
    bool lastLapWasBest;            // Was last lap a new best

    void clear() {
        lapInProgress = false;
        lapStartMs = 0;
        currentLapTimeMs = 0;
        lastLapTimeMs = 0;
        lastLapWasBest = false;
    }
};

// ============================================================
// INITIALIZATION
// ============================================================

/**
 * @brief Initialize the lap manager
 * 
 * Sets up internal state. Call during setup().
 */
void initLapManager();

/**
 * @brief Reset lap manager for new session
 * 
 * Clears lap state but preserves best lap and reference.
 */
void resetLapManager();

// ============================================================
// SESSION MANAGEMENT
// ============================================================

/**
 * @brief Start a new session
 * 
 * Increments session number and resets lap count.
 * 
 * @return New session number
 */
uint16_t startNewSession();

/**
 * @brief Get current session number
 * 
 * @return Session number
 */
uint16_t getCurrentSession();

/**
 * @brief Set session number manually
 * 
 * @param session Session number to set
 */
void setCurrentSession(uint16_t session);

// ============================================================
// LAP CONTROL
// ============================================================

/**
 * @brief Start a new lap
 * 
 * Begins timing for a new lap. Call when crossing finish line.
 * 
 * @param startTimeMs Starting timestamp (millis)
 */
void startLap(unsigned long startTimeMs);

/**
 * @brief Complete current lap
 * 
 * Finalizes lap timing and updates best lap if applicable.
 * 
 * @param finishTimeMs Finish timestamp (millis)
 * @return Lap time in milliseconds
 */
unsigned long completeLap(unsigned long finishTimeMs);

/**
 * @brief Cancel current lap
 * 
 * Abandons current lap timing without recording.
 */
void cancelLap();

/**
 * @brief Check if lap is in progress
 * 
 * @return true if currently timing a lap
 */
bool isLapInProgress();

/**
 * @brief Get elapsed time for current lap
 * 
 * @param nowMs Current time (millis)
 * @return Elapsed lap time in milliseconds
 */
unsigned long getLapElapsedTime(unsigned long nowMs);

// ============================================================
// LAP NUMBER MANAGEMENT
// ============================================================

/**
 * @brief Get current lap number
 * 
 * @return Lap number (1-based)
 */
uint16_t getCurrentLapNumber();

/**
 * @brief Set lap number manually
 * 
 * @param lap Lap number to set
 */
void setCurrentLapNumber(uint16_t lap);

/**
 * @brief Increment lap number
 */
void incrementLapNumber();

// ============================================================
// GPS POINT TRACKING
// ============================================================

/**
 * @brief Update current GPS point
 * 
 * Stores the latest GPS data and shifts previous point.
 * 
 * @param point New GPS point
 */
void updateCurrentPoint(const GPSPoint& point);

/**
 * @brief Get current GPS point
 * 
 * @return Most recent GPS point
 */
const GPSPoint& getCurrentPoint();

/**
 * @brief Get previous GPS point
 * 
 * @return Previous GPS point (for line crossing detection)
 */
const GPSPoint& getPreviousPoint();

/**
 * @brief Clear GPS point history
 */
void clearPointHistory();

// ============================================================
// BEST LAP MANAGEMENT
// ============================================================

/**
 * @brief Get best lap time
 * 
 * @return Best lap time in ms, or UINT32_MAX if none
 */
uint32_t getBestLapTimeMs();

/**
 * @brief Set best lap time manually
 * 
 * @param timeMs Best lap time
 */
void setBestLapTimeMs(uint32_t timeMs);

/**
 * @brief Check if a time is a new best
 * 
 * @param timeMs Lap time to check
 * @return true if this would be a new best lap
 */
bool isNewBestLap(uint32_t timeMs);

/**
 * @brief Get best lap number
 * 
 * @return Lap number of best lap, or 0 if none
 */
uint16_t getBestLapNumber();

/**
 * @brief Check if we have a valid best lap time set
 * 
 * @return true if best lap time is recorded
 */
bool hasBestLapTime();

// ============================================================
// REFERENCE LAP MANAGEMENT
// ============================================================

/**
 * @brief Check if reference lap is available
 * 
 * @return true if reference lap is loaded
 */
bool hasValidReferenceLap();

/**
 * @brief Set whether reference lap is valid
 * 
 * @param valid Validity flag
 */
void setHasValidReferenceLap(bool valid);

/**
 * @brief Load reference lap from storage (SPIFFS)
 * 
 * @return true if loaded successfully
 */
bool loadRefLapFromStorage();

/**
 * @brief Load reference lap from built-in track data
 * 
 * @param lapIdx Index into track data lap boundaries
 * @return true if loaded successfully
 */
bool loadRefLapFromTrackData(int lapIdx);

/**
 * @brief Load reference lap from prior session data
 * 
 * Uses the embedded reference lap data.
 * 
 * @return true if loaded successfully
 */
bool loadReferenceLapFromPriorSession();

/**
 * @brief Save current best lap as reference
 * 
 * @param lap Lap data to save
 * @return true if saved successfully
 */
bool saveAsReferenceLap(const LapData& lap);

// ============================================================
// LAP STATE ACCESS
// ============================================================

/**
 * @brief Get current lap state
 * 
 * @return Const reference to lap state
 */
const LapState& getLapState();

/**
 * @brief Get mutable lap state
 * 
 * @return Reference to lap state
 */
LapState& getLapStateMutable();

// ============================================================
// LAP RECORDING
// ============================================================

/**
 * @brief Start recording GPS points for current lap
 */
void startLapRecording();

/**
 * @brief Add point to current lap recording
 * 
 * @param lat Latitude
 * @param lng Longitude
 * @param lapTimeMs Lap time at this point
 * @param speedKmh Speed at this point
 * @param headingDeg Heading at this point
 */
void recordLapPoint(double lat, double lng, unsigned long lapTimeMs,
                    float speedKmh, float headingDeg);

/**
 * @brief Finish recording and get lap data
 * 
 * @param outLap Output: recorded lap data
 * @return true if recording was valid
 */
bool finishLapRecording(LapData& outLap);

/**
 * @brief Cancel current lap recording
 */
void cancelLapRecording();

/**
 * @brief Check if currently recording
 * 
 * @return true if recording in progress
 */
bool isRecordingLap();

// ============================================================
// LAP VALIDATION
// ============================================================

/**
 * @brief Validate a lap time
 * 
 * Checks if lap time is within reasonable bounds.
 * 
 * @param timeMs Lap time to validate
 * @return true if lap time is valid
 */
bool isValidLapTime(unsigned long timeMs);

/**
 * @brief Get minimum valid lap time
 * 
 * @return Minimum lap time in ms
 */
unsigned long getMinLapTimeMs();

/**
 * @brief Get maximum valid lap time
 * 
 * @return Maximum lap time in ms
 */
unsigned long getMaxLapTimeMs();

#endif // LAP_MANAGER_H