#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <stdint.h>

// Frame start bytes (MSB set to identify frame start)
constexpr uint8_t BEGIN_LFRAME = 0x80;  // Lap frame
constexpr uint8_t BEGIN_GFRAME = 0x81;  // GPS frame
constexpr uint8_t BEGIN_TFRAME = 0x82;  // Time frame

// Ensure consistent struct packing across compilers
#pragma pack(push, 1)

// Lap timing frame
struct lFrame {
    uint32_t time;          // Current lap time in milliseconds
    uint32_t pts;           // Number of reference lap GPS points
    int32_t delta;          // Time delta vs reference in milliseconds (+slower, -faster)
    uint32_t start_time;    // Lap start timestamp (reserved)
    uint32_t finish_time;   // Lap finish timestamp (reserved)
    uint16_t dist_start;    // Distance to nearest reference point in meters
    uint32_t bestLapMs;     // Best lap time in milliseconds
    uint32_t refTimeMs;     // Reference time at current track position in milliseconds
    uint16_t bestLapNumber; // Best lap number (1-based)
    bool hasBestLap;        // Whether we have a valid best lap to compare against

    // Lap complete display state
    uint32_t lastCompletedLapMs;     // Last completed lap time (for 5s display)
    unsigned long lapCompleteDisplayEndMs; // When to stop showing last lap time
};

// GPS data frame
struct gFrame {
    uint8_t sats;           // Number of satellites
    uint8_t fix;            // GPS fix status (0=no fix, 1=fix)
    uint16_t bearing;       // Current bearing in degrees (0-359)
    int32_t lon;            // Longitude * 1e7
    int32_t lat;            // Latitude * 1e7
    uint32_t speed;         // Speed in m/h (km/h * 1000)
};

// Time/status frame
struct tFrame {
    uint8_t seconds;        // GPS time seconds (0-59)
    uint8_t minutes;        // GPS time minutes (0-59)
    uint8_t hours;          // GPS time hours (0-23)
    uint8_t day;            // Day of week (reserved)
    uint8_t date;           // Day of month (1-31)
    uint8_t month;          // Month (1-12)
    uint16_t year;          // Year (e.g., 2025)
    uint16_t lap;           // Current lap number
    uint16_t session;       // Current session number
    uint8_t stat;           // Status bits (reserved)
};

#pragma pack(pop)

#endif // PROTOCOL_HPP
