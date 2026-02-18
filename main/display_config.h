/**
 * @file display_config.h
 * @brief Configurable display pages — loaded from /sdcard/config/display.json
 */

#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

/**
 * What the large 56pt center label shows.
 */
enum class CenterContent {
    DELTA,    // ±delta seconds vs reference lap
    LAPTIME,  // current lap time
    SPEED,    // current speed (km/h integer)
};

/**
 * What the full-width background colour band represents.
 */
enum class BarMode {
    TIME,   // bar width ∝ time delta (±DELTA_BAR_RANGE_SECONDS)
    SPEED,  // bar width ∝ speed delta vs reference (existing behaviour)
    NONE,   // bar hidden
};

struct PageConfig {
    CenterContent center   = CenterContent::DELTA;
    BarMode       bar      = BarMode::TIME;
    bool showSectors       = true;
    bool showBestLaps      = true;
    bool showLapNumber     = true;
    bool showDatetime      = true;
};

static constexpr int MAX_USER_PAGES = 4;

struct DisplayConfig {
    int        pageCount = 2;
    PageConfig pages[MAX_USER_PAGES];
};

/**
 * Load (or re-load) config from /sdcard/config/display.json.
 * Falls back to defaults silently if the file is absent or malformed.
 * Call once after the SD card is mounted.
 */
void loadDisplayConfig();

/**
 * Return the currently active display configuration.
 */
const DisplayConfig& getDisplayConfig();

#endif // DISPLAY_CONFIG_H
