/**
 * @file sd_logger.cpp
 * @brief SD card logging implementation
 */

#include "sd_logger.h"
#include "sdcard_manager.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <sys/stat.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "SDLOG";

// ── Rolling debug log parameters ────────────────────────────
static constexpr size_t  MAX_LOG_BYTES   = 2UL * 1024 * 1024;  // 2 MB per file
static constexpr int     MAX_ROLL_FILES  = 5;                   // keep at most 5 files

static FILE*  s_logFile      = nullptr;
static size_t s_logBytes     = 0;
static int    s_rollCount    = 0;
static char   s_logBasePath[96] = {};  // base without roll-count suffix

// ── Session CSV ──────────────────────────────────────────────
static FILE* s_sessionFile = nullptr;

// ── Helpers ──────────────────────────────────────────────────

static bool mkdirp(const char* path) {
    // Skip stat() — its return value is unreliable on some FATFS VFS builds.
    // Call mkdir() directly and treat EEXIST (already there) as success.
    if (mkdir(path, 0755) == 0) return true;
    if (errno == EEXIST) return true;
    ESP_LOGW(TAG, "mkdir(%s) errno=%d", path, errno);
    return false;
}

/**
 * Fill dateBuf (YYYY-MM-DD) and timeBuf (HH-MM-SS) from system clock (TZ env var 기준 로컬 시각).
 * Falls back to "0000-00-00" / "00-00-00" if time is not set.
 */
static void getLocalTimeStrings(char* dateBuf, int dLen, char* timeBuf, int tLen) {
    time_t now = time(nullptr);
    if (now < 1704067200LL) {  // before 2024-01-01 UTC → time not set
        strncpy(dateBuf, "0000-00-00", dLen);
        strncpy(timeBuf, "00-00-00",   tLen);
        return;
    }
    struct tm tm = {};
    localtime_r(&now, &tm);
    // Cast to unsigned + modulo so the compiler can prove the output length is bounded
    snprintf(dateBuf, dLen, "%04u-%02u-%02u",
             (unsigned)(tm.tm_year + 1900) % 10000u,
             (unsigned)(tm.tm_mon + 1) % 100u,
             (unsigned)tm.tm_mday % 100u);
    snprintf(timeBuf, tLen, "%02u-%02u-%02u",
             (unsigned)tm.tm_hour % 100u,
             (unsigned)tm.tm_min % 100u,
             (unsigned)tm.tm_sec % 100u);
}

// ── Debug log ────────────────────────────────────────────────

static bool openLogFileRolled(void) {
    if (!sdcardIsMounted()) return false;
    mkdirp("/sdcard/logs");

    char path[128];
    if (s_rollCount == 0) {
        snprintf(path, sizeof(path), "%s.log", s_logBasePath);
    } else {
        snprintf(path, sizeof(path), "%s_%d.log", s_logBasePath, s_rollCount);
    }

    s_logFile = fopen(path, "w");
    if (!s_logFile) {
        ESP_LOGW(TAG, "Cannot open log: %s  errno=%d", path, errno);
        return false;
    }
    fprintf(s_logFile,
            "uptime_ms,lat,lng,speed_kmh,heading_deg,fix,sats,state,lap_ms,event\n");
    s_logBytes = 0;
    ESP_LOGI(TAG, "Opened log: %s", path);
    return true;
}

bool sdLoggerInit(void) {
    if (!sdcardIsMounted()) {
        ESP_LOGW(TAG, "SD not mounted — logging disabled");
        return false;
    }
    if (s_logFile) {
        fclose(s_logFile);
        s_logFile = nullptr;
    }
    s_rollCount = 0;

    char dateBuf[16], timeBuf[16];
    getLocalTimeStrings(dateBuf, sizeof(dateBuf), timeBuf, sizeof(timeBuf));
    snprintf(s_logBasePath, sizeof(s_logBasePath),
             "/sdcard/logs/%s_%s", dateBuf, timeBuf);

    return openLogFileRolled();
}

static void rollIfNeeded(void) {
    if (s_logBytes < MAX_LOG_BYTES) return;
    if (s_logFile) { fclose(s_logFile); s_logFile = nullptr; }
    s_rollCount++;
    if (s_rollCount > MAX_ROLL_FILES) s_rollCount = 1;  // wrap, overwrite oldest
    openLogFileRolled();
}

void sdLogGPS(unsigned long ms,
              double lat, double lng,
              float speedKmh, float heading,
              int fixType, int sats,
              const char* state, unsigned long lapMs) {
    if (!s_logFile) return;
    int n = fprintf(s_logFile,
                    "%lu,%.7f,%.7f,%.1f,%.1f,%d,%d,%s,%lu,\n",
                    ms, lat, lng, speedKmh, heading,
                    fixType, sats,
                    state ? state : "?",
                    lapMs);
    if (n > 0) {
        s_logBytes += (size_t)n;
        // Flush every 100 GPS writes (~10s at 10Hz) to ensure data survives power loss
        static uint8_t s_flushCounter = 0;
        if (++s_flushCounter >= 100) {
            fflush(s_logFile);
            s_flushCounter = 0;
        }
    }
    rollIfNeeded();
}

void sdLogEvent(unsigned long ms, const char* tag, const char* msg) {
    if (!s_logFile) return;
    int n = fprintf(s_logFile,
                    "%lu,0.0000000,0.0000000,0.0,0.0,0,0,,0,%s:%s\n",
                    ms,
                    tag ? tag : "",
                    msg ? msg : "");
    if (n > 0) {
        s_logBytes += (size_t)n;
        fflush(s_logFile);  // flush events immediately
    }
    rollIfNeeded();
}

void sdLoggerClose(void) {
    if (s_logFile) {
        fflush(s_logFile);
        fclose(s_logFile);
        s_logFile = nullptr;
        ESP_LOGI(TAG, "Log closed");
    }
}

// ── Session CSV ──────────────────────────────────────────────

bool sdSessionStart(void) {
    if (!sdcardIsMounted()) return false;

    if (s_sessionFile) {
        fclose(s_sessionFile);
        s_sessionFile = nullptr;
    }

    char dateBuf[16], timeBuf[16];
    getLocalTimeStrings(dateBuf, sizeof(dateBuf), timeBuf, sizeof(timeBuf));

    char dirPath[64];
    snprintf(dirPath, sizeof(dirPath), "/sdcard/laps/%s", dateBuf);
    mkdirp("/sdcard/laps");
    mkdirp(dirPath);

    char filePath[128];
    snprintf(filePath, sizeof(filePath), "%s/%s.csv", dirPath, timeBuf);

    s_sessionFile = fopen(filePath, "w");
    if (!s_sessionFile) {
        ESP_LOGW(TAG, "Cannot open session CSV: %s", filePath);
        return false;
    }
    fprintf(s_sessionFile, "lap_num,lap_ms,lat,lng,speed_kmh,heading_deg\n");
    fflush(s_sessionFile);
    ESP_LOGI(TAG, "Session CSV: %s", filePath);
    return true;
}

void sdSessionPoint(uint16_t lapNum, unsigned long lapMs,
                    double lat, double lng,
                    float speedKmh, float heading) {
    if (!s_sessionFile) return;
    fprintf(s_sessionFile, "%u,%lu,%.7f,%.7f,%.1f,%.1f\n",
            lapNum, lapMs, lat, lng, speedKmh, heading);
}

void sdSessionEnd(void) {
    if (s_sessionFile) {
        fflush(s_sessionFile);
        fclose(s_sessionFile);
        s_sessionFile = nullptr;
        ESP_LOGI(TAG, "Session CSV closed");
    }
}
