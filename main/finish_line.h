/**
 * @file finish_line.h
 * @brief Finish line crossing detection for lap timing
 * @version 1.3
 */

#ifndef FINISH_LINE_H
#define FINISH_LINE_H

#include "config.h"  // For MIN_LAP_TIME_MS

struct TrackTemplate;  // Forward declaration

// ============================================================
// Configuration
// ============================================================

constexpr float FINISH_LINE_WIDTH_M = 10.0f;           // Line width in meters
constexpr float MIN_SEGMENT_LENGTH_M = 1.5f;           // Ignore jittery sub-meter moves
// MIN_LAP_TIME_MS is defined in config.h
constexpr unsigned long CROSSING_DEADZONE_MS = 5000;   // 5 second deadzone after crossing
constexpr float MIN_CROSS_SPEED_KMH = 12.0f;           // Reject noise if inferred speed is too low
constexpr float CROSSING_MAX_JUMP_M = 80.0f;           // Ignore improbable GPS jumps (meters)
constexpr float CROSSING_LINE_PROXIMITY_MARGIN = 1.5f; // Allow some slack beyond line width
constexpr float MAX_LINE_OFFSET_M = 25.0f;             // Reject if far away from finish line
constexpr float FINISH_PROXIMITY_M = 25.0f;            // Require rider to be within this distance of line

// ============================================================
// Data Structures
// ============================================================

struct FinishLine {
    double lat1, lng1;  // Start point
    double lat2, lng2;  // End point
    uint16_t validBearingMin;  // Valid heading range min (degrees)
    uint16_t validBearingMax;  // Valid heading range max (degrees)
    bool configured;
};

struct CrossingState {
    unsigned long lastCrossingMs;
    unsigned long prevLapTimeMs;
    bool inDeadzone;
    double prevLat;
    double prevLng;
    bool hasPrevPoint;
};

// ============================================================
// Function Declarations
// ============================================================

// Initialization
void initFinishLine();

// Configuration
bool setFinishLineFromCurrentPos(double lat, double lng, float heading);
bool loadFinishLineFromStorage();
bool saveFinishLineToStorage();
void clearFinishLine();

// Detection
bool checkLineCrossing(double lat, double lng, float heading, unsigned long lapTimeMs);

// State access
bool isFinishLineConfigured();
const FinishLine& getFinishLine();
void resetCrossingState();

#endif // FINISH_LINE_H
