/**
 * @file finish_line.cpp
 * @brief Finish line crossing detection implementation (ESP-IDF)
 * @version 1.0
 */

#include "finish_line.h"
#include "track_types.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <errno.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "geo_utils.h"

static const char *TAG = "FINISH_LINE";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================
// Constants
// ============================================================

constexpr float DEG_TO_RAD_F = M_PI / 180.0f;
constexpr float EARTH_RADIUS_M = 6371000.0f;
constexpr char FINISH_LINE_DIR[] = "/spiffs/config";
constexpr char FINISH_LINE_FILE[] = "/spiffs/config/finish_line.bin";
constexpr char LEGACY_FINISH_LINE_DIR[] = "/config";
constexpr char LEGACY_FINISH_LINE_FILE[] = "/config/finish_line.bin";

// ============================================================
// Global State
// ============================================================

static FinishLine s_finishLine = {};
static CrossingState s_crossingState = {};

// ============================================================
// Helper Functions
// ============================================================

static bool fileExists(const char* path) {
    struct stat st = {};
    return stat(path, &st) == 0;
}

static bool ensureDirectory(const char* path) {
    struct stat st = {};
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    if (mkdir(path, 0755) == 0) {
        return true;
    }
    if (errno == ENOTSUP) {
        return true;
    }
    ESP_LOGE(TAG, "mkdir failed for %s (errno=%d)", path, errno);
    return false;
}

static void migrateLegacyFinishLine() {
    if (!fileExists(LEGACY_FINISH_LINE_FILE)) {
        return;
    }

    ensureDirectory(FINISH_LINE_DIR);
    if (rename(LEGACY_FINISH_LINE_FILE, FINISH_LINE_FILE) == 0) {
        ESP_LOGI(TAG, "Finish line config migrated to /spiffs");
    }
}

// segmentsIntersect() is provided by geo_utils.h

/**
 * Normalize bearing to 0-360 range
 */
static uint16_t normalizeBearing(int bearing) {
    while (bearing < 0) bearing += 360;
    while (bearing >= 360) bearing -= 360;
    return (uint16_t)bearing;
}

/**
 * Check if bearing is within valid range (handles wrap-around at 0/360)
 */
static bool isBearingValid(float heading, uint16_t minBearing, uint16_t maxBearing) {
    uint16_t h = normalizeBearing((int)heading);

    if (minBearing <= maxBearing) {
        // Normal range (e.g., 80-100)
        return h >= minBearing && h <= maxBearing;
    } else {
        // Wrap-around range (e.g., 350-10)
        return h >= minBearing || h <= maxBearing;
    }
}

// ============================================================
// Public Functions
// ============================================================

void initFinishLine() {
    s_finishLine = {};
    s_crossingState = {};

    ensureDirectory(FINISH_LINE_DIR);
    migrateLegacyFinishLine();
    loadFinishLineFromStorage();
}

bool setFinishLineFromCurrentPos(double lat, double lng, float heading) {
    // Create a perpendicular line through current position
    // Line extends FINISH_LINE_WIDTH_M / 2 in each direction

    float perpHeading = normalizeBearing((int)heading + 90);
    float perpRad = perpHeading * DEG_TO_RAD_F;

    // Distance in degrees (approximate)
    float distDeg = (FINISH_LINE_WIDTH_M / 2.0f) / 111000.0f;

    // Calculate endpoints
    s_finishLine.lat1 = lat + distDeg * cosf(perpRad);
    s_finishLine.lng1 = lng + distDeg * sinf(perpRad) / cosf(lat * DEG_TO_RAD_F);
    s_finishLine.lat2 = lat - distDeg * cosf(perpRad);
    s_finishLine.lng2 = lng - distDeg * sinf(perpRad) / cosf(lat * DEG_TO_RAD_F);

    // Set valid bearing range (+/- 45 degrees from current heading)
    s_finishLine.validBearingMin = normalizeBearing((int)heading - 45);
    s_finishLine.validBearingMax = normalizeBearing((int)heading + 45);
    s_finishLine.configured = true;

    // Reset crossing state
    resetCrossingState();

    ESP_LOGI(TAG, "Finish line set at %.6f,%.6f heading %.0f", lat, lng, heading);

    return saveFinishLineToStorage();
}

bool loadFinishLineFromStorage() {
    if (!fileExists(FINISH_LINE_FILE)) {
        return false;
    }

    FILE* f = fopen(FINISH_LINE_FILE, "rb");
    if (!f) {
        return false;
    }

    size_t read = fread(&s_finishLine, 1, sizeof(FinishLine), f);
    fclose(f);

    if (read != sizeof(FinishLine)) {
        s_finishLine = {};
        return false;
    }

    ESP_LOGI(TAG, "Finish line loaded from storage");
    return s_finishLine.configured;
}

bool saveFinishLineToStorage() {
    ensureDirectory(FINISH_LINE_DIR);

    FILE* f = fopen(FINISH_LINE_FILE, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to save finish line");
        return false;
    }

    size_t written = fwrite(&s_finishLine, 1, sizeof(FinishLine), f);
    fclose(f);

    return written == sizeof(FinishLine);
}

void clearFinishLine() {
    s_finishLine = {};
    resetCrossingState();
    remove(FINISH_LINE_FILE);
    ESP_LOGI(TAG, "Finish line cleared");
}

bool checkLineCrossing(double lat, double lng, float heading, unsigned long lapTimeMs) {
    if (!s_finishLine.configured) {
        return false;
    }

    unsigned long now = (unsigned long)(esp_timer_get_time() / 1000ULL);

    // Check deadzone
    if (s_crossingState.inDeadzone) {
        if (now - s_crossingState.lastCrossingMs < CROSSING_DEADZONE_MS) {
            return false;
        }
        s_crossingState.inDeadzone = false;
    }

    // Check minimum lap time
    if (lapTimeMs < MIN_LAP_TIME_MS) {
        // Still update previous point
        s_crossingState.prevLat = lat;
        s_crossingState.prevLng = lng;
        s_crossingState.hasPrevPoint = true;
        return false;
    }

    // Check bearing validity
    if (!isBearingValid(heading, s_finishLine.validBearingMin, s_finishLine.validBearingMax)) {
        s_crossingState.prevLat = lat;
        s_crossingState.prevLng = lng;
        s_crossingState.hasPrevPoint = true;
        return false;
    }

    // Need previous point for crossing detection
    if (!s_crossingState.hasPrevPoint) {
        s_crossingState.prevLat = lat;
        s_crossingState.prevLng = lng;
        s_crossingState.hasPrevPoint = true;
        return false;
    }

    // Check if movement vector crosses finish line
    bool crossed = segmentsIntersect(
        s_crossingState.prevLat, s_crossingState.prevLng,
        lat, lng,
        s_finishLine.lat1, s_finishLine.lng1,
        s_finishLine.lat2, s_finishLine.lng2
    );

    // Update previous point
    s_crossingState.prevLat = lat;
    s_crossingState.prevLng = lng;

    if (crossed) {
        s_crossingState.lastCrossingMs = now;
        s_crossingState.inDeadzone = true;
        ESP_LOGI(TAG, "*** FINISH LINE CROSSED ***");
        return true;
    }

    return false;
}

bool setFinishLineFromDefinition(const FinishLineDefinition& def) {
    if (!def.isConfigured()) {
        ESP_LOGW(TAG, "Finish line definition not configured");
        return false;
    }

    s_finishLine.lat1 = def.lat1;
    s_finishLine.lng1 = def.lng1;
    s_finishLine.lat2 = def.lat2;
    s_finishLine.lng2 = def.lng2;
    s_finishLine.validBearingMin = (uint16_t)def.validHeadingMin;
    s_finishLine.validBearingMax = (uint16_t)def.validHeadingMax;
    s_finishLine.configured = true;

    resetCrossingState();

    ESP_LOGI(TAG, "Finish line set from track definition (%.6f,%.6f → %.6f,%.6f)",
             def.lat1, def.lng1, def.lat2, def.lng2);
    return true;
}

bool checkFirstLineCrossing(double lat, double lng, float heading) {
    // 최초 스타트라인 통과 감지: MIN_LAP_TIME / deadzone 체크 없이 교차만 검사
    // 세션 시작(startSession) 직전에만 사용
    if (!s_finishLine.configured) {
        return false;
    }

    if (!isBearingValid(heading, s_finishLine.validBearingMin, s_finishLine.validBearingMax)) {
        s_crossingState.prevLat = lat;
        s_crossingState.prevLng = lng;
        s_crossingState.hasPrevPoint = true;
        return false;
    }

    if (!s_crossingState.hasPrevPoint) {
        s_crossingState.prevLat = lat;
        s_crossingState.prevLng = lng;
        s_crossingState.hasPrevPoint = true;
        return false;
    }

    bool crossed = segmentsIntersect(
        s_crossingState.prevLat, s_crossingState.prevLng,
        lat, lng,
        s_finishLine.lat1, s_finishLine.lng1,
        s_finishLine.lat2, s_finishLine.lng2
    );

    s_crossingState.prevLat = lat;
    s_crossingState.prevLng = lng;

    if (crossed) {
        unsigned long now = (unsigned long)(esp_timer_get_time() / 1000ULL);
        s_crossingState.lastCrossingMs = now;
        s_crossingState.inDeadzone = true;
        ESP_LOGI(TAG, "*** SESSION START: first line crossing ***");
        return true;
    }

    return false;
}

bool isFinishLineConfigured() {
    return s_finishLine.configured;
}

const FinishLine& getFinishLine() {
    return s_finishLine;
}

void resetCrossingState() {
    s_crossingState = {};
}
