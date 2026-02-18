/**
 * @file track_manager.h
 * @brief Track detection and selection management
 * @version 1.0
 * 
 * Manages track auto-detection based on GPS position and allows
 * manual selection of tracks and layouts.
 */

#ifndef TRACK_MANAGER_H
#define TRACK_MANAGER_H

#include "track_types.h"
#include "builtin_tracks.h"

// ============================================================
// INITIALIZATION
// ============================================================

/**
 * @brief Initialize the track manager
 * 
 * Sets up internal state and prepares for track detection.
 * Should be called once during setup().
 */
void initTrackManager();

/**
 * @brief Reset track manager state
 * 
 * Clears current track selection and detection state.
 */
void resetTrackManager();

// ============================================================
// TRACK DETECTION
// ============================================================

/**
 * @brief Attempt to detect track based on GPS position
 * 
 * Searches built-in tracks to find one whose center is within
 * detection radius of the given position.
 * 
 * @param lat Current latitude
 * @param lng Current longitude
 * @return Pointer to detected track, or nullptr if none found
 */
const TrackDefinition* detectTrackByPosition(double lat, double lng);

/**
 * @brief Check if we're still within the active track's area
 * 
 * @param lat Current latitude
 * @param lng Current longitude
 * @return true if still within track detection radius
 */
bool isWithinActiveTrack(double lat, double lng);

/**
 * @brief Get distance to nearest known track
 * 
 * @param lat Current latitude
 * @param lng Current longitude
 * @param outTrack Output: nearest track (can be nullptr)
 * @return Distance in meters to nearest track center
 */
float getDistanceToNearestTrack(double lat, double lng, const TrackDefinition** outTrack);

// ============================================================
// ACTIVE TRACK MANAGEMENT
// ============================================================

/**
 * @brief Get the currently active track state
 * 
 * @return Reference to active track state
 */
ActiveTrack& getActiveTrack();

/**
 * @brief Get the currently active track state (const)
 * 
 * @return Const reference to active track state
 */
const ActiveTrack& getActiveTrackConst();

/**
 * @brief Check if a track is currently active
 * 
 * @return true if a track is selected
 */
bool hasActiveTrack();

/**
 * @brief Set the active track
 * 
 * @param track Track to set as active
 * @param layoutIndex Layout index to use (default: 0)
 * @return true if successful
 */
bool setActiveTrack(const TrackDefinition* track, int layoutIndex = 0);

/**
 * @brief Set the active track by ID
 * 
 * @param trackId Track ID (e.g., "everland")
 * @param layoutId Optional layout ID (nullptr for default)
 * @return true if track was found and set
 */
bool setActiveTrackById(const char* trackId, const char* layoutId = nullptr);

/**
 * @brief Clear the active track
 */
void clearActiveTrack();

/**
 * @brief Mark the active track as user-confirmed
 * 
 * This prevents auto-switching to another detected track.
 */
void confirmActiveTrack();

// ============================================================
// LAYOUT SELECTION
// ============================================================

/**
 * @brief Set the active layout for current track
 * 
 * @param layoutIndex Index of layout to select
 * @return true if successful
 */
bool setActiveLayout(int layoutIndex);

/**
 * @brief Set the active layout by ID
 * 
 * @param layoutId Layout ID (e.g., "full", "short")
 * @return true if layout was found and set
 */
bool setActiveLayoutById(const char* layoutId);

/**
 * @brief Switch to next layout (wraps around)
 * 
 * @return true if successful
 */
bool nextLayout();

/**
 * @brief Switch to previous layout (wraps around)
 * 
 * @return true if successful
 */
bool prevLayout();

/**
 * @brief Get number of layouts for active track
 * 
 * @return Number of layouts, or 0 if no track active
 */
int getActiveLayoutCount();

/**
 * @brief Get current layout index
 * 
 * @return Current layout index, or -1 if no track active
 */
int getActiveLayoutIndex();

// ============================================================
// FINISH LINE ACCESS
// ============================================================

/**
 * @brief Get the finish line for active layout
 * 
 * @return Pointer to finish line, or nullptr if no track active
 */
const FinishLineDefinition* getActiveFinishLine();

/**
 * @brief Check if finish line is configured
 * 
 * @return true if finish line is available
 */
bool hasActiveFinishLine();

// ============================================================
// SECTOR ACCESS
// ============================================================

/**
 * @brief Get number of sectors in active layout
 * 
 * @return Number of sectors, or 0 if none
 */
int getActiveSectorCount();

/**
 * @brief Get sector by index
 * 
 * @param index Sector index (0-based)
 * @return Pointer to sector, or nullptr if invalid
 */
const Sector* getActiveSector(int index);

/**
 * @brief Check if active layout has sectors
 * 
 * @return true if sectors are defined
 */
bool hasActiveSectors();

// ============================================================
// REFERENCE LAP ACCESS
// ============================================================

/**
 * @brief Check if active layout has built-in reference lap
 * 
 * @return true if reference data is available
 */
bool hasBuiltinReference();

/**
 * @brief Get built-in reference lap info
 * 
 * @param outStartIdx Output: start index in track points
 * @param outEndIdx Output: end index in track points
 * @param outTimeMs Output: reference lap time
 * @return true if reference info was retrieved
 */
bool getBuiltinReferenceInfo(int& outStartIdx, int& outEndIdx, uint32_t& outTimeMs);

// ============================================================
// PROXIMITY DETECTION (with hysteresis)
// ============================================================

/**
 * @brief Update track proximity state using 2-stage detection + hysteresis
 *
 * Stage 1: Bounding box pre-filter (no trig, very cheap)
 * Stage 2: Haversine distance (only when Stage 1 passes)
 * Hysteresis: enter at 1× detectionRadiusM, exit at 1.5× detectionRadiusM
 *
 * Also auto-sets active track when proximity is first detected.
 *
 * @param lat Current latitude
 * @param lng Current longitude
 * @return true if within proximity zone of any track
 */
bool updateTrackProximity(double lat, double lng);

/**
 * @brief Check if currently near any track (cached from last updateTrackProximity)
 * @return true if within proximity zone
 */
bool isNearAnyTrack();

/**
 * @brief Get name of the track currently in proximity (or nullptr)
 */
const char* getNearTrackName();

// ============================================================
// TRACK LISTING
// ============================================================

/**
 * @brief Get number of available tracks (built-in)
 * 
 * @return Number of tracks
 */
int getAvailableTrackCount();

/**
 * @brief Get track by index
 * 
 * @param index Track index
 * @return Pointer to track, or nullptr if invalid
 */
const TrackDefinition* getAvailableTrack(int index);

/**
 * @brief Get track name by index
 * 
 * @param index Track index
 * @return Track name, or "Unknown" if invalid
 */
const char* getAvailableTrackName(int index);

#endif // TRACK_MANAGER_H