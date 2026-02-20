/**
 * @file track_types.h
 * @brief Track, Layout, Sector, and FinishLineDefinition data structures
 * @version 1.0
 * 
 * Defines the hierarchical structure for track management:
 * - TrackDefinition: A racing circuit (e.g., Everland Speedway)
 * - TrackLayout: A specific configuration of the track (e.g., Full, Short)
 * - Sector: A segment of the track for split timing
 * - FinishLineDefinition: Start/finish line coordinates
 */

#ifndef TRACK_TYPES_H
#define TRACK_TYPES_H

#include <cstdint>
#include <cstring>
#include "config.h"

// Forward declare TrackPoint and LapBoundary - defined in everland_track_data.h
// These are used for built-in reference lap data
struct TrackPoint;
struct LapBoundary;

// ============================================================
// FINISH LINE
// ============================================================

/**
 * @brief Finish line definition using two GPS coordinates
 * 
 * The finish line is defined as a line segment between two points.
 * Crossing is detected when the car's path intersects this segment
 * while traveling in a valid heading direction.
 */
struct FinishLineDefinition {
    double lat1;                // Point A latitude
    double lng1;                // Point A longitude
    double lat2;                // Point B latitude
    double lng2;                // Point B longitude
    
    // Valid heading range for crossing (to prevent reverse triggers)
    float validHeadingMin;      // Minimum valid heading (degrees, 0-360)
    float validHeadingMax;      // Maximum valid heading (degrees, 0-360)
    
    /**
     * @brief Check if finish line is configured
     */
    bool isConfigured() const {
        return (lat1 != 0.0 || lng1 != 0.0) && (lat2 != 0.0 || lng2 != 0.0);
    }
    
    /**
     * @brief Clear finish line
     */
    void clear() {
        lat1 = lat2 = lng1 = lng2 = 0.0;
        validHeadingMin = 0.0f;
        validHeadingMax = 360.0f;
    }
};

// ============================================================
// SECTOR
// ============================================================

/**
 * @brief Sector definition for split timing
 * 
 * A sector is a segment of the track. Sector times allow comparing
 * performance in different parts of the track, not just overall lap time.
 * 
 * Sectors can be defined either by:
 * 1. Boundary lines (like finish line) - more accurate
 * 2. Track distance range - simpler, uses cumulative distance
 */
struct Sector {
    const char* name;           // Display name ("S1", "Sector 1", etc.)
    
    // Boundary line definition (optional - if both points are 0, use distance)
    double boundaryLat1;
    double boundaryLng1;
    double boundaryLat2;
    double boundaryLng2;
    
    // Valid heading range for sector boundary crossing
    float validHeadingMin;
    float validHeadingMax;
    
    // Track distance definition (alternative to boundary line)
    float startDistanceM;       // Start distance from track start (meters)
    float endDistanceM;         // End distance from track start (meters)
    
    /**
     * @brief Check if sector uses boundary line detection
     */
    bool usesBoundaryLine() const {
        return (boundaryLat1 != 0.0 || boundaryLng1 != 0.0);
    }
    
    /**
     * @brief Check if sector uses distance-based detection
     */
    bool usesDistance() const {
        return !usesBoundaryLine() && endDistanceM > startDistanceM;
    }
};

// ============================================================
// TRACK LAYOUT
// ============================================================

/**
 * @brief A specific configuration/layout of a track
 * 
 * Many tracks have multiple configurations:
 * - Full course vs short course
 * - Clockwise vs counter-clockwise
 * - With/without chicanes
 * 
 * Each layout has its own finish line, sectors, and reference data.
 */
struct TrackLayout {
    const char* id;             // Unique identifier ("full", "short", "reverse")
    const char* name;           // Display name ("Full Course", "Short Course")
    
    // Finish line for this layout
    FinishLineDefinition finishLine;
    
    // Lap time validation
    uint32_t minLapTimeMs;      // Minimum valid lap time (reject faster)
    uint32_t maxLapTimeMs;      // Maximum valid lap time (reject slower)
    
    // Sector definitions
    int sectorCount;            // Number of sectors (0 = no sectors)
    const Sector* sectors;      // Array of sectors (nullptr if sectorCount == 0)
    
    // Built-in reference lap data (optional)
    bool hasBuiltinReference;   // Is reference data embedded in firmware?
    int builtinRefStartIdx;     // Start index in track_points array
    int builtinRefEndIdx;       // End index in track_points array
    uint32_t builtinRefTimeMs;  // Reference lap time
    
    // Minimum speed for finish line crossing (track identification)
    float minCrossSpeedKmh;     // Minimum GPS speed to trigger crossing (km/h)

    // Track length
    float trackLengthM;         // Total track length in meters
    
    /**
     * @brief Check if layout has sectors defined
     */
    bool hasSectors() const {
        return sectorCount > 0 && sectors != nullptr;
    }
    
    /**
     * @brief Check if layout has built-in reference
     */
    bool hasReference() const {
        return hasBuiltinReference && builtinRefEndIdx > builtinRefStartIdx;
    }
    
    /**
     * @brief Get sector by index
     */
    const Sector* getSector(int index) const {
        if (index < 0 || index >= sectorCount || sectors == nullptr) {
            return nullptr;
        }
        return &sectors[index];
    }
};

// ============================================================
// TRACK DEFINITION
// ============================================================

/**
 * @brief Complete track/circuit definition
 *
 * Represents a racing circuit with one or more layouts.
 * Track is identified by finish line crossing (no proximity detection).
 */
struct TrackDefinition {
    const char* id;             // Unique identifier ("everland", "inje")
    const char* name;           // Display name ("Everland Speedway")
    const char* country;        // Country code ("KR", "JP", "US")

    // Layouts
    int layoutCount;            // Number of layouts
    const TrackLayout* layouts; // Array of layouts
    
    /**
     * @brief Get layout by index
     */
    const TrackLayout* getLayout(int index) const {
        if (index < 0 || index >= layoutCount || layouts == nullptr) {
            return nullptr;
        }
        return &layouts[index];
    }
    
    /**
     * @brief Get layout by ID
     */
    const TrackLayout* getLayoutById(const char* layoutId) const {
        if (layoutId == nullptr || layouts == nullptr) {
            return nullptr;
        }
        for (int i = 0; i < layoutCount; i++) {
            if (strcmp(layouts[i].id, layoutId) == 0) {
                return &layouts[i];
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Get default layout (first one)
     */
    const TrackLayout* getDefaultLayout() const {
        return getLayout(0);
    }
};

// ============================================================
// ACTIVE TRACK STATE
// ============================================================

/**
 * @brief Currently active track and layout
 * 
 * Runtime state tracking which track/layout is currently selected.
 */
struct ActiveTrack {
    const TrackDefinition* track;   // Selected track (nullptr if none)
    const TrackLayout* layout;      // Selected layout (nullptr if none)
    int layoutIndex;                // Index of selected layout
    bool userConfirmed;             // Has user confirmed this selection?

    /**
     * @brief Check if a track is active
     */
    bool isValid() const {
        return track != nullptr && layout != nullptr;
    }

    /**
     * @brief Clear active track
     */
    void clear() {
        track = nullptr;
        layout = nullptr;
        layoutIndex = -1;
        userConfirmed = false;
    }
    
    /**
     * @brief Get track name (or "Unknown")
     */
    const char* getTrackName() const {
        return track ? track->name : "Unknown";
    }
    
    /**
     * @brief Get layout name (or "Default")
     */
    const char* getLayoutName() const {
        return layout ? layout->name : "Default";
    }
    
    /**
     * @brief Get finish line for current layout
     */
    const FinishLineDefinition* getFinishLineDefinition() const {
        return layout ? &layout->finishLine : nullptr;
    }
    
    /**
     * @brief Get sector count for current layout
     */
    int getSectorCount() const {
        return layout ? layout->sectorCount : 0;
    }
};

// NOTE: TrackPoint and LapBoundary structs are defined in everland_track_data.h
// They are forward-declared at the top of this file for reference.

#endif // TRACK_TYPES_H