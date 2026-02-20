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
 * 3. Define the track (id, name, country, layouts)
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
        
        // Lap time validation
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
        
        // Minimum crossing speed
        60.0f,                      // minCrossSpeedKmh

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

    // Layouts
    LAYOUT_COUNT,                   // layoutCount
    LAYOUTS                         // layouts
};

} // namespace everland

// ============================================================
// INJE SPEEDIUM (인제스피디움)
// ============================================================

namespace inje {

// Sector boundary coordinates
// Section 1 (S1/S2): 38°00'15.07"N 128°17'38.27"E
constexpr double SECTOR_1_2_LAT = 38.004186;
constexpr double SECTOR_1_2_LNG = 128.293964;
// Section 2 (S2/S3): 37°59'55.67"N 128°17'30.99"E
constexpr double SECTOR_2_3_LAT = 37.998797;
constexpr double SECTOR_2_3_LNG = 128.291942;
// Section 3 (S3/S4): 37°59'59.64"N 128°17'09.33"E
constexpr double SECTOR_3_4_LAT = 37.999900;
constexpr double SECTOR_3_4_LNG = 128.285925;

constexpr int SECTOR_BOUNDARY_COUNT = 3;

struct SectorBoundaryCoord {
    double lat;
    double lng;
};

constexpr SectorBoundaryCoord SECTOR_BOUNDARIES[SECTOR_BOUNDARY_COUNT] = {
    {SECTOR_1_2_LAT, SECTOR_1_2_LNG},
    {SECTOR_2_3_LAT, SECTOR_2_3_LNG},
    {SECTOR_3_4_LAT, SECTOR_3_4_LNG},
};

// Finish line coordinates (measured on-site)
// P1: 38°00'02.10"N 128°17'28.00"E  →  38.000583, 128.291111
// P2: 38°00'02.02"N 128°17'29.46"E  →  38.000561, 128.291517
// Line runs nearly E-W (~35.7m), cars cross heading roughly N (340°~60°)
constexpr double FINISH_LINE_A_LAT = 38.000583;  // western end
constexpr double FINISH_LINE_A_LNG = 128.291111;
constexpr double FINISH_LINE_B_LAT = 38.000561;  // eastern end
constexpr double FINISH_LINE_B_LNG = 128.291517;

// Sector definitions (4 sectors with boundary line detection)
// Boundary lines are perpendicular cuts at each sector boundary GPS point
static const Sector SECTORS[] = {
    {
        "S1",                       // name
        // S1/S2 boundary line (perpendicular to track bearing ~47°, cut at 137°)
        38.004285, 128.293848,     // boundaryLat1, boundaryLng1 (NW side)
        38.004088, 128.294081,     // boundaryLat2, boundaryLng2 (SE side)
        20.0f, 75.0f,              // valid heading: cars going NE ~47°
        0.0f, 0.0f                 // distance not used (boundary line mode)
    },
    {
        "S2",                       // name
        // S2/S3 boundary line (perpendicular to track bearing ~197°, cut at 107°)
        37.998836, 128.291779,     // boundaryLat1, boundaryLng1 (W side)
        37.998758, 128.292106,     // boundaryLat2, boundaryLng2 (E side)
        170.0f, 225.0f,            // valid heading: cars going SSW ~197°
        0.0f, 0.0f
    },
    {
        "S3",                       // name
        // S3/S4 boundary line (perpendicular to track bearing ~283°, cut at 13°)
        38.000031, 128.285964,     // boundaryLat1, boundaryLng1 (N side)
        37.999769, 128.285887,     // boundaryLat2, boundaryLng2 (S side)
        255.0f, 315.0f,            // valid heading: cars going WNW ~283°
        0.0f, 0.0f
    },
    {
        "S4",                       // name
        // Last sector: ends at finish line (no boundary line needed)
        0.0, 0.0, 0.0, 0.0,       // no boundary line
        0.0f, 360.0f,              // any heading (finish line detection handles it)
        0.0f, 0.0f
    }
};

static constexpr int SECTOR_COUNT = sizeof(SECTORS) / sizeof(SECTORS[0]);

// Layout definitions
static const TrackLayout LAYOUTS[] = {
    {
        // Full Course
        "full",                     // id
        "Full Course",              // name

        // Finish line
        {
            FINISH_LINE_A_LAT,      // lat1
            FINISH_LINE_A_LNG,      // lng1
            FINISH_LINE_B_LAT,      // lat2
            FINISH_LINE_B_LNG,      // lng2
            340.0f,                 // validHeadingMin (northward, wraps around 0°)
            60.0f                   // validHeadingMax
        },

        // Lap time validation (Inje Speedium: ~3.908km)
        60000,                      // minLapTimeMs (1 minute)
        300000,                     // maxLapTimeMs (5 minutes)

        // Sectors
        SECTOR_COUNT,               // sectorCount
        SECTORS,                    // sectors

        // No built-in reference lap
        false,                      // hasBuiltinReference
        0,                          // builtinRefStartIdx
        0,                          // builtinRefEndIdx
        0,                          // builtinRefTimeMs

        // Minimum crossing speed
        60.0f,                      // minCrossSpeedKmh

        // Track length: Inje Speedium 3.908km
        3908.0f                     // trackLengthM
    }
};

static constexpr int LAYOUT_COUNT = sizeof(LAYOUTS) / sizeof(LAYOUTS[0]);

// Track definition
static const TrackDefinition TRACK = {
    "inje",                         // id
    "Inje Speedium",                // name (인제스피디움)
    "KR",                           // country

    // Layouts
    LAYOUT_COUNT,                   // layoutCount
    LAYOUTS                         // layouts
};

} // namespace inje

// ============================================================
// TEST TRACK
// ============================================================

namespace ipark10 {

// Finish line coordinates
// P1: 37°18'39.66"N 127°05'09.46"E  →  37.311017, 127.085961
// P2: 37°18'39.35"N 127°05'10.05"E  →  37.310931, 127.086125
// Line runs NW-SE (~118°), cars cross heading NNE (~28°)
constexpr double FINISH_LINE_A_LAT = 37.311017;
constexpr double FINISH_LINE_A_LNG = 127.085961;
constexpr double FINISH_LINE_B_LAT = 37.310931;
constexpr double FINISH_LINE_B_LNG = 127.086125;

// Layout definitions
static const TrackLayout LAYOUTS[] = {
    {
        "full",                     // id
        "Full Course",              // name

        // Finish line
        {
            FINISH_LINE_A_LAT,      // lat1
            FINISH_LINE_A_LNG,      // lng1
            FINISH_LINE_B_LAT,      // lat2
            FINISH_LINE_B_LNG,      // lng2
            350.0f,                 // validHeadingMin (NNE direction, wraps around 0°)
            60.0f                   // validHeadingMax
        },

        // Lap time validation (test track: generous limits)
        10000,                      // minLapTimeMs (10 seconds)
        600000,                     // maxLapTimeMs (10 minutes)

        // No sectors
        0,                          // sectorCount
        nullptr,                    // sectors

        // No built-in reference lap
        false,                      // hasBuiltinReference
        0,                          // builtinRefStartIdx
        0,                          // builtinRefEndIdx
        0,                          // builtinRefTimeMs

        // Minimum crossing speed (walking speed for testing)
        2.0f,                       // minCrossSpeedKmh

        // Track length unknown
        0.0f                        // trackLengthM
    }
};

static constexpr int LAYOUT_COUNT = sizeof(LAYOUTS) / sizeof(LAYOUTS[0]);

// Track definition
static const TrackDefinition TRACK = {
    "ipark10",                      // id
    "iPark 10",                     // name
    "KR",                           // country

    // Layouts
    LAYOUT_COUNT,                   // layoutCount
    LAYOUTS                         // layouts
};

} // namespace ipark10

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
    &inje::TRACK,
    &ipark10::TRACK,
    // Add more tracks here:
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