/**
 * @file gps_processor.h
 * @brief GPS Hardware mode for GPS Lap Timer
 * @version 1.0
 * 
 * Handles real GPS data from u-blox module for actual lap timing.
 * This is the primary production mode for live track use.
 */

#ifndef GPS_PROCESSOR_H
#define GPS_PROCESSOR_H

#include <cstdint>

// ============================================================
// GPS PROCESSOR STATE
// ============================================================

/**
 * @brief State management for GPS hardware mode
 */
struct GPSProcessorState {
    unsigned long lapStartMs = 0;       // When current lap started (millis)
    unsigned long lastValidGpsMs = 0;   // Last valid GPS update time
    bool lapStarted = false;            // Is lap timing active
    bool gpsSignalLost = false;         // GPS timeout occurred
    
    /**
     * @brief Reset state to initial values
     */
    void reset() {
        lapStartMs = 0;
        lastValidGpsMs = 0;
        lapStarted = false;
        gpsSignalLost = false;
    }
};

// ============================================================
// PUBLIC INTERFACE
// ============================================================

/**
 * @brief Initialize GPS hardware mode
 * 
 * Sets up GPS module communication and initializes state.
 * Should be called before processRealGPS().
 */
void initializeGPSMode();

/**
 * @brief Process one GPS update cycle
 * 
 * Called from main loop. Reads GPS data from u-blox module,
 * updates position, calculates delta, detects finish line crossing,
 * and handles lap completion.
 * 
 * Features:
 * - GPS signal timeout detection
 * - Position recording for lap storage
 * - Delta calculation vs reference lap
 * - Finish line crossing detection
 */
void processRealGPS();

/**
 * @brief Reset GPS mode state
 * 
 * Clears lap timing, stops recording, and resets crossing detection.
 * Call this to start a fresh GPS session.
 */
void resetRealGPS();

// Note: isGPSSignalLost() is defined in geo/gps_filter.h
// Note: isLapInProgress() is defined in timing/lap_manager.h

/**
 * @brief Get last valid GPS timestamp
 * @return millis() value of last valid GPS update
 */
unsigned long getLastValidGpsTimeMs();

/**
 * @brief Get current GPS processor state (read-only)
 */
const GPSProcessorState& getGPSProcessorState();

/**
 * @brief Get mutable GPS processor state
 */
GPSProcessorState& getGPSProcessorStateMutable();

// Forward declaration for sector boundary access
struct SectorBoundaryPoint;

/**
 * @brief Get GPS mode sector boundary coordinates
 */
const SectorBoundaryPoint* getGPSSectorBoundaries();

/**
 * @brief Get number of GPS mode sector boundaries
 */
int getGPSSectorBoundaryCount();

#endif // GPS_PROCESSOR_H