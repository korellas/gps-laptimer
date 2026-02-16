/**
 * @file dead_reckoning.h
 * @brief Dead reckoning for GPS signal loss
 * @version 1.0
 * 
 * When GPS signal is lost, this module estimates position based on
 * last known speed and heading. This allows continuous lap timing
 * during brief signal dropouts.
 */

#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include "types.h"
#include "config.h"

// ============================================================
// DEAD RECKONING STATE
// ============================================================

/**
 * @brief Dead reckoning state information
 */
struct DRState {
    bool active = false;                // Is DR currently active
    unsigned long startMs = 0;          // When DR started
    unsigned long lastUpdateMs = 0;     // Last DR update time
    unsigned long totalDurationMs = 0;  // How long DR has been running
    
    // Starting conditions (from last valid GPS)
    GPSPoint lastKnownPoint;            // Last valid GPS position
    float lastSpeedKmh = 0.0f;          // Speed at signal loss
    float lastHeadingDeg = 0.0f;        // Heading at signal loss
    
    // Current estimated position
    GPSPoint estimatedPoint;            // Current DR estimate
    
    // Quality metrics
    float confidence = 1.0f;            // Decreases over time (0-1)
    float estimatedErrorM = 0.0f;       // Estimated position error (meters)
};

// ============================================================
// CONFIGURATION
// ============================================================

/**
 * @brief Dead reckoning configuration
 */
struct DRConfig {
    // Enable/disable
    bool enabled = ENABLE_DEAD_RECKONING;
    
    // Maximum duration to use DR (ms)
    unsigned long maxDurationMs = MAX_DEAD_RECKONING_MS;
    
    // Minimum speed to start DR (km/h)
    // Below this, position estimation is unreliable
    float minSpeedKmh = MIN_DEAD_RECKONING_SPEED_KMH;
    
    // Confidence decay rate (per second)
    // Confidence = 1.0 - (duration * decayRate)
    float confidenceDecayPerSec = 0.15f;  // Lose 15% confidence per second
    
    // Error growth rate (meters per second)
    // Estimated error = duration * errorGrowthRate
    float errorGrowthMPerSec = 2.0f;  // ~2m error per second
};

// ============================================================
// PUBLIC API
// ============================================================

/**
 * @brief Initialize dead reckoning module
 * 
 * @param config Configuration (uses defaults if not specified)
 */
void initDeadReckoning(const DRConfig& config = DRConfig());

/**
 * @brief Reset dead reckoning state
 * 
 * Stops any active DR and clears state.
 */
void resetDeadReckoning();

/**
 * @brief Start dead reckoning from last known position
 * 
 * Call this when GPS signal is lost. DR will estimate
 * position based on last speed and heading.
 * 
 * @param lastPoint Last valid GPS point
 * @param speedKmh Speed at signal loss
 * @param headingDeg Heading at signal loss
 * @param nowMs Current timestamp (millis())
 * @return true if DR started successfully
 */
bool startDeadReckoning(const GPSPoint& lastPoint, 
                        float speedKmh, 
                        float headingDeg,
                        unsigned long nowMs);

/**
 * @brief Update dead reckoning estimate
 * 
 * Call this periodically while GPS is unavailable.
 * Returns the estimated current position.
 * 
 * @param nowMs Current timestamp (millis())
 * @return Estimated GPS point (check isDeadReckoningActive() for validity)
 */
GPSPoint updateDeadReckoning(unsigned long nowMs);

/**
 * @brief Stop dead reckoning
 * 
 * Call this when GPS signal is recovered.
 */
void stopDeadReckoning();

/**
 * @brief Check if dead reckoning is currently active
 * 
 * @return true if DR is running
 */
bool isDeadReckoningActive();

/**
 * @brief Check if DR has exceeded maximum duration
 * 
 * @return true if DR has been running too long
 */
bool isDeadReckoningExpired();

/**
 * @brief Get current DR state
 * 
 * @return Current state structure
 */
const DRState& getDeadReckoningState();

/**
 * @brief Get current confidence level
 * 
 * @return Confidence (0-1), decreases over time
 */
float getDeadReckoningConfidence();

/**
 * @brief Get estimated position error
 * 
 * @return Estimated error in meters
 */
float getDeadReckoningError();

// ============================================================
// CONFIGURATION
// ============================================================

/**
 * @brief Update DR configuration
 * 
 * @param config New configuration
 */
void setDRConfig(const DRConfig& config);

/**
 * @brief Get current DR configuration
 * 
 * @return Current configuration
 */
DRConfig getDRConfig();

#endif // DEAD_RECKONING_H