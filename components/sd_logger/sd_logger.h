/**
 * @file sd_logger.h
 * @brief SD card logging for GPS lap timer diagnostics
 *
 * Two log streams:
 * 1) Debug event log: /sdcard/logs/YYYYMMDD_HHMMSS.log (rolling, max 2MB)
 *    - Every GPS fix, state transitions, lap/finish events
 * 2) Session GPS CSV: /sdcard/laps/YYYY-MM-DD/HH-MM-SS.csv
 *    - Continuous GPS track for the entire session (all laps)
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Debug event log (rolling) ──────────────────────────────

/**
 * Open the debug log file.  Call once after SD is mounted.
 * Returns true on success.
 */
bool sdLoggerInit(void);

/**
 * Log a raw GPS point (called for every valid GPS fix).
 * @param ms       System uptime in milliseconds
 * @param lat/lng  WGS-84 position
 * @param speedKmh GPS speed
 * @param heading  Heading in degrees
 * @param fixType  u-blox fixType (0=no fix, 2=2D, 3=3D)
 * @param sats     Satellites used
 * @param state    Session state string ("PRE_TRACK"/"NEAR_TRACK"/"SESSION_ACTIVE")
 * @param lapMs    Current lap time in ms (0 if not in session)
 */
void sdLogGPS(unsigned long ms,
              double lat, double lng,
              float speedKmh, float heading,
              int fixType, int sats,
              const char* state, unsigned long lapMs);

/**
 * Log a key event (finish line, lap complete, track detect, etc.).
 * Flushed to disk immediately so it survives crashes.
 */
void sdLogEvent(unsigned long ms, const char* tag, const char* msg);

/** Close and flush the debug log. */
void sdLoggerClose(void);

// ── Per-session GPS CSV ─────────────────────────────────────

/**
 * Start a new session CSV file at /sdcard/laps/YYYY-MM-DD/HH-MM-SS.csv.
 * Call when the lap timer session begins (first finish-line crossing).
 * Returns true on success.
 */
bool sdSessionStart(void);

/**
 * Append a GPS point to the current session CSV.
 * @param lapNum  Current lap number (1-based)
 * @param lapMs   Elapsed time within current lap (ms)
 */
void sdSessionPoint(uint16_t lapNum, unsigned long lapMs,
                    double lat, double lng,
                    float speedKmh, float heading);

/** Close the session CSV (end of session or power-off). */
void sdSessionEnd(void);

#ifdef __cplusplus
}
#endif
