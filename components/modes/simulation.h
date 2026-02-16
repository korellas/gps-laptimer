/**
 * @file simulation.h
 * @brief Simulation mode for GPS Lap Timer
 * @version 1.0
 * 
 * Provides simulated GPS data playback using pre-recorded track data.
 * Used for development and testing without real GPS hardware.
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include "esp_timer.h"
#include <cstdint>
#include "types.h"
#include "config.h"
#include "../../examples/everland_track_data.h"

// ============================================================
// SIMULATION STATE
// ============================================================

/**
 * @brief State management for simulation mode
 */
struct SimulationState {
    int currentLapIdx = 0;              // Which lap in lap_boundaries we're playing
    int currentPointIdx = 0;            // Current point index within the lap
    unsigned long startMillis = 0;       // When this lap started (real time)
    unsigned long lastUpdateMs = 0;      // Last simulation update time
    unsigned long lapCompleteTimeMs = 0; // When lap completed (for suppressing delta)
    bool initialized = false;            // Has simulation been initialized
    
    /**
     * @brief Reset state for new lap (keeps currentLapIdx)
     */
    void reset() {
        currentPointIdx = 0;
        startMillis = (unsigned long)(esp_timer_get_time() / 1000ULL);
        lastUpdateMs = 0;
        initialized = false;
        lapCompleteTimeMs = (unsigned long)(esp_timer_get_time() / 1000ULL);
    }
    
    /**
     * @brief Advance to next lap in data
     */
    void startNextLap() {
        currentLapIdx = (currentLapIdx + 1) % NUM_LAPS;
        reset();
    }
    
    /**
     * @brief Check if delta display should be suppressed
     * 
     * Suppress delta for first 2 seconds after lap reset to avoid jumps
     */
    bool isDeltaSuppressed() const {
        return ((unsigned long)(esp_timer_get_time() / 1000ULL) - lapCompleteTimeMs) <
               DELTA_SUPPRESS_AFTER_RESET_MS;
    }
    
    /**
     * @brief Get current lap boundaries from track data
     */
    const LapBoundary& getCurrentLapBounds() const {
        return lap_boundaries[currentLapIdx];
    }
};

// ============================================================
// PUBLIC INTERFACE
// ============================================================

/**
 * @brief Initialize simulation mode
 * 
 * Sets up simulation state, loads reference lap (from prior session or best sample),
 * and prepares first lap for playback.
 */
void initializeSimulation();

/**
 * @brief Process one simulation update cycle
 * 
 * Called from main loop. Updates simulation state based on elapsed time,
 * processes GPS data points, calculates delta, and checks for lap completion.
 * Respects SIM_UPDATE_INTERVAL_MS for timing.
 */
void processSimulation();

/**
 * @brief Reset simulation to initial state
 * 
 * Resets lap counter to 1, clears all state, and reinitializes.
 * Call this to start a fresh simulation session.
 */
void resetSimulation();

/**
 * @brief Handle lap completion in simulation mode
 * 
 * @param lapTimeMs Lap time in milliseconds
 * 
 * Updates best lap tracking, loads new reference if needed,
 * advances to next lap, and resets state for new lap.
 */
void onSimLapComplete(unsigned long lapTimeMs);

/**
 * @brief Load GPS points for a specific lap into internal buffer
 *
 * @param lapIdx Index of lap in lap_boundaries array
 * @return true if loaded successfully, false otherwise
 */
bool loadSimulationLap(int lapIdx);

/**
 * @brief Get current simulation state (read-only access)
 */
const SimulationState& getSimulationState();

/**
 * @brief Get mutable simulation state
 */
SimulationState& getSimulationStateMutable();

/**
 * @brief Get current simulation lap data index
 */
int getCurrentSimLapDataIndex();

/**
 * @brief Get best simulation lap data index (-1 if none)
 */
int getBestSimLapDataIndex();

/**
 * @brief Get current simulation lap points
 */
const GPSPoint* getSimLapPoints();

/**
 * @brief Get current simulation lap point count
 */
int getSimLapPointCount();

/**
 * @brief Get index of best lap in sample data (-1 if none)
 */
int getBestLapDataIdx();

/**
 * @brief Set best lap data index
 */
void setBestLapDataIdx(int idx);

#endif // SIMULATION_H
