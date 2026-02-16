/**
 * @file builtin_tracks.h
 * @brief Built-in track definitions
 * @version 1.0
 * 
 * Contains pre-defined track data that is compiled into the firmware.
 * Currently includes Everland Speedway with full course layout.
 * 
 * To add a new track:
 * 1. Define sectors (optional)
 * 2. Define layouts with finish lines
 * 3. Define the track with center coordinates
 * 4. Add to BUILTIN_TRACKS array
 */

#ifndef BUILTIN_TRACKS_H
#define BUILTIN_TRACKS_H

#include <cstring>
#include "../../examples/everland_track_data.h"
#include "../../examples/everland_reference_5283.h"
#include "track_types.h"

// ============================================================
// EVERLAND SPEEDWAY
// ============================================================

namespace everland {

// Sector boundary coordinates (physical points on track)
constexpr double SECTOR_1_2_LAT = 37.2974004;
constexpr double SECTOR_1_2_LNG = 127.2175125;
constexpr double SECTOR_2_3_LAT = 37.2961782;
constexpr double SECTOR_2_3_LNG = 127.2138076;

constexpr int SECTOR_BOUNDARY_COUNT = 2;

struct SectorBoundaryCoord {
    double lat;
    double lng;
};

constexpr SectorBoundaryCoord SECTOR_BOUNDARIES[SECTOR_BOUNDARY_COUNT] = {
    {SECTOR_1_2_LAT, SECTOR_1_2_LNG},
    {SECTOR_2_3_LAT, SECTOR_2_3_LNG},
};

// Sector definitions for Everland full course
// Sectors divide the track for split timing
// Distances are approximate defaults; recalculated from reference lap at runtime
static const Sector SECTORS[] = {
    {
        "S1",                       // name
        0.0, 0.0, 0.0, 0.0,        // boundary line (not used - using distance)
        0.0f, 360.0f,              // valid heading (any)
        0.0f, 6121.0f              // distance: 0m - 6121m (start to first sector point)
    },
    {
        "S2",                       // name
        0.0, 0.0, 0.0, 0.0,        // boundary line (not used)
        0.0f, 360.0f,              // valid heading
        6121.0f, 11382.0f          // distance: 6121m - 11382m (first to second sector point)
    },
    {
        "S3",                       // name
        0.0, 0.0, 0.0, 0.0,        // boundary line (not used)
        0.0f, 360.0f,              // valid heading
        11382.0f, 12881.0f         // distance: 11382m - finish (second sector point to end)
    }
};

static constexpr int SECTOR_COUNT = sizeof(SECTORS) / sizeof(SECTORS[0]);

// Layout definitions
static const TrackLayout LAYOUTS[] = {
    {
        // Full Course - the standard layout
        "full",                     // id
        "Full Course",              // name
        
        // Finish line (from everland_track_data.h)
        {
            FINISH_LINE_A_LAT,      // lat1
            FINISH_LINE_A_LNG,      // lng1
            FINISH_LINE_B_LAT,      // lat2
            FINISH_LINE_B_LNG,      // lng2
            315.0f,                 // validHeadingMin (NW direction)
            45.0f                   // validHeadingMax (allowing wrap around 0)
        },
        
        // Timing expectations
        52000,                      // expectedLapTimeMs (~52 seconds for fast lap)
        30000,                      // minLapTimeMs (30 seconds - anything faster is invalid)
        180000,                     // maxLapTimeMs (3 minutes - anything slower is invalid)
        
        // Sectors
        SECTOR_COUNT,               // sectorCount
        SECTORS,                    // sectors
        
        // Built-in reference lap (from everland_reference_5283.h)
        true,                       // hasBuiltinReference
        0,                          // builtinRefStartIdx
        everland_reference_5283::REF_TOTAL_TRACK_POINTS - 1,  // builtinRefEndIdx
        everland_reference_5283::ref_lap_boundaries[0].lapTimeMs,  // builtinRefTimeMs
        
        // Track length (approximate) - 12.88km based on GPS data
        12881.0f                    // trackLengthM
    }
};

static constexpr int LAYOUT_COUNT = sizeof(LAYOUTS) / sizeof(LAYOUTS[0]);

// Track definition
static const TrackDefinition TRACK = {
    "everland",                     // id
    "Everland Speedway",            // name
    "KR",                           // country
    
    // Center coordinates (from everland_track_data.h)
    TRACK_CENTER_LAT,               // centerLat
    TRACK_CENTER_LNG,               // centerLng
    1200.0f,                        // detectionRadiusM (1.2km)
    
    // Layouts
    LAYOUT_COUNT,                   // layoutCount
    LAYOUTS                         // layouts
};

} // namespace everland

// ============================================================
// BUILT-IN TRACKS REGISTRY
// ============================================================

/**
 * @brief Array of all built-in tracks
 * 
 * Add new tracks here as they are defined.
 */
static const TrackDefinition* const BUILTIN_TRACKS[] = {
    &everland::TRACK,
    // Add more tracks here:
    // &inje::TRACK,
    // &taebaek::TRACK,
};

static constexpr int BUILTIN_TRACK_COUNT = sizeof(BUILTIN_TRACKS) / sizeof(BUILTIN_TRACKS[0]);

// ============================================================
// HELPER FUNCTIONS
// ============================================================

/**
 * @brief Get built-in track by index
 * 
 * @param index Track index (0 to BUILTIN_TRACK_COUNT-1)
 * @return Pointer to track definition, or nullptr if invalid
 */
inline const TrackDefinition* getBuiltinTrack(int index) {
    if (index < 0 || index >= BUILTIN_TRACK_COUNT) {
        return nullptr;
    }
    return BUILTIN_TRACKS[index];
}

/**
 * @brief Get built-in track by ID
 * 
 * @param id Track ID (e.g., "everland")
 * @return Pointer to track definition, or nullptr if not found
 */
inline const TrackDefinition* getBuiltinTrackById(const char* id) {
    if (id == nullptr) {
        return nullptr;
    }
    for (int i = 0; i < BUILTIN_TRACK_COUNT; i++) {
        if (strcmp(BUILTIN_TRACKS[i]->id, id) == 0) {
            return BUILTIN_TRACKS[i];
        }
    }
    return nullptr;
}

/**
 * @brief Get total number of built-in tracks
 * 
 * @return Number of tracks
 */
inline int getBuiltinTrackCount() {
    return BUILTIN_TRACK_COUNT;
}

// ============================================================
// EVERLAND TRACK DATA ACCESS
// ============================================================

/**
 * @brief Get Everland track points array
 * 
 * These are the raw GPS points from recorded laps.
 * Used for simulation and reference lap loading.
 */
inline const TrackPoint* getEverlandTrackPoints() {
    return track_points;
}

/**
 * @brief Get Everland track points count
 */
inline int getEverlandTrackPointCount() {
    return TOTAL_TRACK_POINTS;
}

/**
 * @brief Get Everland lap boundaries
 */
inline const LapBoundary* getEverlandLapBoundaries() {
    return lap_boundaries;
}

/**
 * @brief Get Everland lap count
 */
inline int getEverlandLapCount() {
    return NUM_LAPS;
}

/**
 * @brief Get Everland reference lap points
 */
inline const everland_reference_5283::TrackPoint* getEverlandReferencePoints() {
    return everland_reference_5283::ref_track_points;
}

/**
 * @brief Get Everland reference lap point count
 */
inline int getEverlandReferencePointCount() {
    return everland_reference_5283::REF_TOTAL_TRACK_POINTS;
}

#endif // BUILTIN_TRACKS_H