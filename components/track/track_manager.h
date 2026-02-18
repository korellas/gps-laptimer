/**
 * @file track_manager.h
 * @brief Track selection and active track management
 * @version 2.0
 *
 * Manages active track/layout state. Track identification is done by
 * finish line crossing in gps_processor.cpp (PRE_TRACK state).
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
const FinishLineDefinition* getActiveFinishLineDefinition();

/**
 * @brief Check if finish line is configured
 * 
 * @return true if finish line is available
 */
bool hasActiveFinishLineDefinition();

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
// TRACK/LAYOUT ID ACCESS (for lap storage)
// ============================================================

/**
 * @brief Get active track's index in BUILTIN_TRACKS[]
 * @return Index (0-based), or 0xFF if no active track
 */
uint8_t getActiveTrackIndex();

/**
 * @brief Get active layout index within the track
 * @return Index (0-based), or 0xFF if no active track
 */
uint8_t getActiveTrackLayoutIndex();

/**
 * @brief Get active track's string ID (e.g., "everland")
 * @return Track ID, or nullptr if no active track
 */
const char* getActiveTrackId();

/**
 * @brief Get active layout's string ID (e.g., "full")
 * @return Layout ID, or nullptr if no active track/layout
 */
const char* getActiveLayoutId();

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