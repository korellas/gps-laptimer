/**
 * @file lap_storage.h
 * @brief Lap data storage for SPIFFS
 * @version 1.0
 */

#ifndef LAP_STORAGE_H
#define LAP_STORAGE_H

#include <cstddef>
#include <cstdint>
#include <vector>

// ============================================================
// Storage Configuration
// ============================================================

#include "config.h"  // For MAX_LAPS_PER_SESSION, MAX_SESSIONS

constexpr uint32_t LAP_FILE_MAGIC = 0x4C415001;  // "LAP" + version 1
constexpr int MAX_POINTS_PER_LAP = 1000;
// MAX_LAPS_PER_SESSION and MAX_SESSIONS are defined in config.h

// ============================================================
// Compact Data Structures (for storage)
// ============================================================

#pragma pack(push, 1)

struct StoredPoint {
    int32_t lat;       // lat * 1e7
    int32_t lng;       // lng * 1e7
    uint32_t lapTimeMs;
    uint16_t speedX10; // speed * 10 (0-6553.5 km/h)
    uint8_t heading;   // 0-255 = 0-360 degrees
};

struct LapHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t pointCount;
    uint32_t totalTimeMs;
    uint32_t startTimestamp;  // Unix epoch
    uint16_t maxSpeedX10;
    uint16_t avgSpeedX10;
    uint16_t sessionId;
    uint16_t lapId;
    uint8_t reserved[8];
};

#pragma pack(pop)

// ============================================================
// Runtime Data Structures
// ============================================================

struct LapInfo {
    uint16_t sessionId;
    uint16_t lapId;
    uint32_t totalTimeMs;
    uint16_t pointCount;
    float maxSpeedKmh;
};

struct StorableLap {
    std::vector<StoredPoint> points;
    uint32_t totalTimeMs;
    uint32_t startTimestamp;
    float maxSpeedKmh;
    float avgSpeedKmh;
    uint16_t sessionId;
    uint16_t lapId;
};

// ============================================================
// Function Declarations
// ============================================================

// Initialization
bool initLapStorage();

// Save/Load
bool saveLap(const StorableLap& lap);
bool loadLap(StorableLap& lap, uint16_t sessionId, uint16_t lapId);
bool deleteLap(uint16_t sessionId, uint16_t lapId);

// Best lap
bool saveBestLap(const StorableLap& lap);
bool loadBestLap(StorableLap& lap);
bool hasBestLap();
uint32_t getBestLapTime();

// Listing
int listLaps(uint16_t sessionId, LapInfo* outList, int maxCount);
int getSessionCount();
uint16_t getNextSessionId();

// Recording helpers
void startRecordingLap(uint16_t sessionId, uint16_t lapId);
void addPointToRecording(double lat, double lng, unsigned long lapTimeMs,
                          float speedKmh, float headingDeg);
bool finishRecordingLap(StorableLap& outLap);
void cancelRecording();
bool isRecording();

// Cleanup
bool clearAllLaps();
bool clearSession(uint16_t sessionId);

// Storage info
size_t getUsedStorage();
size_t getFreeStorage();

#endif // LAP_STORAGE_H
