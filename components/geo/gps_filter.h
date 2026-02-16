/**
 * @file gps_filter.h
 * @brief GPS data filtering and spike detection
 * @version 1.0
 * 
 * Filters GPS data to handle:
 * - Spike detection (sudden position jumps)
 * - Position smoothing (moving average)
 * - Signal quality assessment
 */

#ifndef GPS_FILTER_H
#define GPS_FILTER_H

#include "types.h"
#include "config.h"

// ============================================================
// FILTER CONFIGURATION
// ============================================================

/**
 * @brief GPS filter configuration parameters
 */
struct GPSFilterConfig {
    // Spike detection
    float maxJumpDistanceM = GPS_MAX_JUMP_DISTANCE_M;   // Max allowed position jump
    float jumpSpeedMultiplier = GPS_JUMP_SPEED_MULTIPLIER; // Speed-based threshold
    
    // Smoothing
    bool enableSmoothing = GPS_ENABLE_SMOOTHING;
    int smoothingWindowSize = GPS_SMOOTHING_WINDOW;
    
    // Dead reckoning trigger
    unsigned long signalLostThresholdMs = GPS_TIMEOUT_MS;
};

// ============================================================
// FILTER STATE
// ============================================================

/**
 * @brief Internal state of the GPS filter
 */
struct GPSFilterState {
    // Last valid point
    GPSPoint lastValidPoint;
    unsigned long lastValidMs = 0;
    
    // History buffer for smoothing
    static constexpr int MAX_HISTORY = 5;
    GPSPoint history[MAX_HISTORY];
    int historyIndex = 0;
    int historyCount = 0;
    
    // Statistics
    int totalPoints = 0;
    int spikeCount = 0;
    int smoothedCount = 0;
    
    // Signal status
    bool signalLost = false;
    unsigned long signalLostSinceMs = 0;
};

// ============================================================
// SPIKE DETECTION RESULT
// ============================================================

/**
 * @brief Result of spike detection check
 */
struct SpikeCheckResult {
    bool isSpike = false;           // Is this point a spike?
    float jumpDistanceM = 0.0f;     // Distance from last valid point
    float expectedMaxM = 0.0f;      // Maximum expected distance based on speed
    float confidence = 1.0f;        // Confidence in the point (0-1)
};

// ============================================================
// PUBLIC API
// ============================================================

/**
 * @brief Initialize the GPS filter
 * 
 * @param config Filter configuration (uses defaults if not specified)
 */
void initGPSFilter(const GPSFilterConfig& config = GPSFilterConfig());

/**
 * @brief Reset the GPS filter state
 * 
 * Clears history and statistics. Call when starting a new session.
 */
void resetGPSFilter();

/**
 * @brief Process a raw GPS point through the filter
 * 
 * This is the main filtering function. It:
 * 1. Checks for spikes and rejects bad data
 * 2. Applies smoothing if enabled
 * 3. Updates internal state
 * 
 * @param raw Raw GPS point from receiver
 * @param nowMs Current timestamp (millis())
 * @return Filtered GPS point with metadata
 */
FilteredGPSPoint filterGPSPoint(const GPSPoint& raw, unsigned long nowMs);

/**
 * @brief Check if a point is likely a spike
 * 
 * Compares the point against the last valid position and
 * determines if the jump is physically impossible.
 * 
 * @param point Point to check
 * @param lastValid Last known valid point
 * @param elapsedMs Time since last valid point
 * @return Spike check result with details
 */
SpikeCheckResult checkForSpike(const GPSPoint& point, 
                                const GPSPoint& lastValid,
                                unsigned long elapsedMs);

/**
 * @brief Apply smoothing to a GPS point
 * 
 * Uses moving average of recent points to reduce noise.
 * 
 * @param point Point to smooth
 * @return Smoothed point
 */
GPSPoint smoothPoint(const GPSPoint& point);

/**
 * @brief Check if GPS signal is considered lost
 * 
 * @return true if no valid data received within timeout
 */
bool isGPSSignalLost();

/**
 * @brief Get time since last valid GPS data
 * 
 * @return Milliseconds since last valid point, or 0 if never received
 */
unsigned long getTimeSinceLastValid();

/**
 * @brief Get the last valid GPS point
 * 
 * @return Last known good position
 */
GPSPoint getLastValidPoint();

// ============================================================
// CONFIGURATION
// ============================================================

/**
 * @brief Update filter configuration
 * 
 * @param config New configuration
 */
void setGPSFilterConfig(const GPSFilterConfig& config);

/**
 * @brief Get current filter configuration
 * 
 * @return Current configuration
 */
GPSFilterConfig getGPSFilterConfig();

// ============================================================
// STATISTICS
// ============================================================

/**
 * @brief Get filter state for debugging/display
 * 
 * @return Current filter state
 */
const GPSFilterState& getGPSFilterState();

/**
 * @brief Get spike rate (spikes / total points)
 * 
 * @return Spike rate as percentage (0-100)
 */
float getSpikeRate();

#endif // GPS_FILTER_H