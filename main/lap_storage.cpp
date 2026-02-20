/**
 * @file lap_storage.cpp
 * @brief Lap data storage implementation using SPIFFS (ESP-IDF)
 * @version 2.0
 */

#include "lap_storage.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "sdcard_manager.h"

static const char *TAG = "LAP_STORAGE";

// ============================================================
// Constants
// ============================================================

constexpr char SPIFFS_LAPS_DIR[] = "/spiffs/laps";
constexpr char SD_LAPS_DIR[] = "/sdcard/laps";
constexpr char LEGACY_LAPS_DIR[] = "/laps";
constexpr char LEGACY_BEST_LAP_FILE[] = "/laps/best.bin";
constexpr int FILENAME_BUF_SIZE = 300;

// 부트 시 1회 결정, 세션 중 변경 불가 (SD fallback 금지 정책)
static const char* s_cachedLapsDir = nullptr;

static const char* getLapsDir() {
    if (!s_cachedLapsDir) {
        s_cachedLapsDir = sdcardIsMounted() ? SD_LAPS_DIR : SPIFFS_LAPS_DIR;
    }

    // 세션 중 SD 탈락 감지 (fallback 하지 않음)
    if (s_cachedLapsDir == SD_LAPS_DIR && !sdcardIsMounted()) {
        ESP_LOGE(TAG, "SD card removed during session! Storage writes will fail.");
    }

    return s_cachedLapsDir;
}

static const char* getLegacyBestLapPath() {
    static char buf[64];
    snprintf(buf, sizeof(buf), "%s/best.bin", getLapsDir());
    return buf;
}

static const char* getBestLapPathForTrack(const char* trackId, const char* layoutId) {
    static char buf[128];
    snprintf(buf, sizeof(buf), "%s/best_%s_%s.bin", getLapsDir(), trackId, layoutId);
    return buf;
}

// ============================================================
// Recording State
// ============================================================

static struct {
    std::vector<StoredPoint> points;
    uint16_t sessionId;
    uint16_t lapId;
    uint32_t startTimestamp;
    float maxSpeedKmh;
    float minSpeedKmh;
    float totalSpeed;
    uint8_t trackIndex;
    uint8_t layoutIndex;
    bool active;
} s_recording = {};

// ============================================================
// Helper Functions
// ============================================================

static void getLapFilename(char* buf, uint16_t sessionId, uint16_t lapId) {
    snprintf(buf, FILENAME_BUF_SIZE, "%s/s%03u_l%03u.bin", getLapsDir(), sessionId, lapId);
}

[[maybe_unused]] static void getSessionDirname(char* buf, uint16_t sessionId) {
    snprintf(buf, FILENAME_BUF_SIZE, "%s/s%03u", getLapsDir(), sessionId);
}

static bool fileExists(const char* path) {
    struct stat st = {};
    return stat(path, &st) == 0;
}

static bool dirExists(const char* path) {
    struct stat st = {};
    return stat(path, &st) == 0 && S_ISDIR(st.st_mode);
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
        // SPIFFS doesn't support mkdir; treat as success.
        return true;
    }
    ESP_LOGE(TAG, "mkdir failed for %s (errno=%d)", path, errno);
    return false;
}

static void migrateLegacyLaps() {
    const char* bestPath = getLegacyBestLapPath();
    if (fileExists(LEGACY_BEST_LAP_FILE) && !fileExists(bestPath)) {
        ensureDirectory(getLapsDir());
        if (rename(LEGACY_BEST_LAP_FILE, bestPath) == 0) {
            ESP_LOGI(TAG, "Best lap migrated to %s", getLapsDir());
        }
    }

    if (!dirExists(LEGACY_LAPS_DIR)) {
        return;
    }

    ensureDirectory(getLapsDir());

    DIR* dir = opendir(LEGACY_LAPS_DIR);
    if (!dir) {
        return;
    }

    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
        const char* name = ent->d_name;
        if (name[0] == '.') {
            continue;
        }

        char oldPath[FILENAME_BUF_SIZE];
        char newPath[FILENAME_BUF_SIZE];
        snprintf(oldPath, sizeof(oldPath), "%s/%s", LEGACY_LAPS_DIR, name);
        snprintf(newPath, sizeof(newPath), "%s/%s", getLapsDir(), name);
        if (rename(oldPath, newPath) != 0) {
            // 크로스 파일시스템 rename 실패 시 복사 후 삭제
            FILE* src = fopen(oldPath, "rb");
            FILE* dst = fopen(newPath, "wb");
            if (src && dst) {
                char cpBuf[256];
                size_t n;
                while ((n = fread(cpBuf, 1, sizeof(cpBuf), src)) > 0) {
                    fwrite(cpBuf, 1, n, dst);
                }
                fclose(src); fclose(dst);
                remove(oldPath);
            } else {
                if (src) fclose(src);
                if (dst) fclose(dst);
                ESP_LOGW(TAG, "Migration failed: %s", oldPath);
            }
        }
    }

    closedir(dir);
}

// ============================================================
// Initialization
// ============================================================

bool initLapStorage() {
    size_t total = 0;
    size_t used = 0;
    if (esp_spiffs_info(nullptr, &total, &used) != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS not mounted or info unavailable");
        return false;
    }

    const char* lapsDir = getLapsDir();
    if (!ensureDirectory(lapsDir)) {
        ESP_LOGE(TAG, "Failed to prepare %s", lapsDir);
        return false;
    }
    migrateLegacyLaps();

    if (sdcardIsMounted()) {
        uint64_t sdFree = sdcardGetFreeBytes();
        uint64_t sdTotal = sdcardGetTotalBytes();
        ESP_LOGI(TAG, "Lap storage: SD card (%llu/%llu MB free)",
                 sdFree / (1024*1024), sdTotal / (1024*1024));
    } else {
        ESP_LOGI(TAG, "Lap storage: SPIFFS fallback");
    }
    ESP_LOGI(TAG, "SPIFFS: %u/%u bytes (%.1f%% free)",
           (unsigned int)used,
           (unsigned int)total,
           100.0f * (float)(total - used) / (float)total);

    return true;
}

// ============================================================
// Save/Load Operations
// ============================================================

bool saveLap(const StorableLap& lap) {
    if (lap.points.empty()) {
        return false;
    }

    char filename[FILENAME_BUF_SIZE];
    getLapFilename(filename, lap.sessionId, lap.lapId);

    FILE* f = fopen(filename, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create %s", filename);
        return false;
    }

    LapHeader header = {};
    header.magic = LAP_FILE_MAGIC;
    header.version = 2;
    header.pointCount = lap.points.size();
    header.totalTimeMs = lap.totalTimeMs;
    header.startTimestamp = lap.startTimestamp;
    header.maxSpeedX10 = (uint16_t)(lap.maxSpeedKmh * 10);
    header.avgSpeedX10 = (uint16_t)(lap.avgSpeedKmh * 10);
    header.sessionId = lap.sessionId;
    header.lapId = lap.lapId;
    header.trackIndex = lap.trackIndex;
    header.layoutIndex = lap.layoutIndex;

    size_t written = fwrite(&header, 1, sizeof(LapHeader), f);
    if (written != sizeof(LapHeader)) {
        fclose(f);
        remove(filename);
        return false;
    }

    written = fwrite(lap.points.data(), 1, lap.points.size() * sizeof(StoredPoint), f);
    fclose(f);

    if (written != lap.points.size() * sizeof(StoredPoint)) {
        remove(filename);
        return false;
    }

    ESP_LOGI(TAG, "Saved lap S%u L%u: %u pts, %.2fs (track=%u layout=%u)",
           lap.sessionId, lap.lapId, (unsigned int)lap.points.size(),
           lap.totalTimeMs / 1000.0f, header.trackIndex, header.layoutIndex);
    return true;
}

// Common helper: read a StorableLap from an already-opened file
// Validates header magic and pointCount upper bound (MAX_POINTS_PER_LAP)
static bool readLapFromFile(FILE* f, StorableLap& lap) {
    LapHeader header;
    size_t rd = fread(&header, 1, sizeof(LapHeader), f);
    if (rd != sizeof(LapHeader) || header.magic != LAP_FILE_MAGIC) {
        return false;
    }

    if (header.pointCount == 0 || header.pointCount > MAX_POINTS_PER_LAP) {
        ESP_LOGW(TAG, "Invalid pointCount: %u (max=%d)", header.pointCount, MAX_POINTS_PER_LAP);
        return false;
    }

    lap.points.resize(header.pointCount);
    rd = fread(lap.points.data(), 1, header.pointCount * sizeof(StoredPoint), f);
    if (rd != header.pointCount * sizeof(StoredPoint)) {
        lap.points.clear();
        return false;
    }

    lap.totalTimeMs = header.totalTimeMs;
    lap.startTimestamp = header.startTimestamp;
    lap.maxSpeedKmh = header.maxSpeedX10 / 10.0f;
    lap.avgSpeedKmh = header.avgSpeedX10 / 10.0f;
    lap.sessionId = header.sessionId;
    lap.lapId = header.lapId;
    if (header.version >= 2) {
        lap.trackIndex = header.trackIndex;
        lap.layoutIndex = header.layoutIndex;
    } else {
        lap.trackIndex = 0xFF;
        lap.layoutIndex = 0xFF;
    }

    return true;
}

bool loadLap(StorableLap& lap, uint16_t sessionId, uint16_t lapId) {
    char filename[FILENAME_BUF_SIZE];
    getLapFilename(filename, sessionId, lapId);

    if (!fileExists(filename)) {
        return false;
    }

    FILE* f = fopen(filename, "rb");
    if (!f) {
        return false;
    }

    bool ok = readLapFromFile(f, lap);
    fclose(f);
    return ok;
}

bool deleteLap(uint16_t sessionId, uint16_t lapId) {
    char filename[FILENAME_BUF_SIZE];
    getLapFilename(filename, sessionId, lapId);
    return remove(filename) == 0;
}

// ============================================================
// Best Lap Operations
// ============================================================

static bool loadBestLapFromFile(StorableLap& lap, const char* path) {
    if (!fileExists(path)) {
        return false;
    }

    FILE* f = fopen(path, "rb");
    if (!f) {
        return false;
    }

    bool ok = readLapFromFile(f, lap);
    fclose(f);
    return ok;
}

bool saveBestLap(const StorableLap& lap, const char* trackId, const char* layoutId) {
    if (lap.points.empty() || !trackId || !layoutId) {
        return false;
    }

    const char* path = getBestLapPathForTrack(trackId, layoutId);
    FILE* f = fopen(path, "wb");
    if (!f) {
        return false;
    }

    LapHeader header = {};
    header.magic = LAP_FILE_MAGIC;
    header.version = 2;
    header.pointCount = lap.points.size();
    header.totalTimeMs = lap.totalTimeMs;
    header.startTimestamp = lap.startTimestamp;
    header.maxSpeedX10 = (uint16_t)(lap.maxSpeedKmh * 10);
    header.avgSpeedX10 = (uint16_t)(lap.avgSpeedKmh * 10);
    header.sessionId = lap.sessionId;
    header.lapId = lap.lapId;
    header.trackIndex = lap.trackIndex;
    header.layoutIndex = lap.layoutIndex;

    size_t written = fwrite(&header, 1, sizeof(LapHeader), f);
    if (written != sizeof(LapHeader)) {
        fclose(f);
        remove(path);
        return false;
    }

    written = fwrite(lap.points.data(), 1, lap.points.size() * sizeof(StoredPoint), f);
    fclose(f);

    if (written != lap.points.size() * sizeof(StoredPoint)) {
        remove(path);
        return false;
    }

    ESP_LOGI(TAG, "NEW BEST LAP [%s/%s]: %.2fs", trackId, layoutId, lap.totalTimeMs / 1000.0f);
    return true;
}

bool loadBestLap(StorableLap& lap, const char* trackId, const char* layoutId) {
    if (!trackId || !layoutId) {
        return false;
    }

    // Try per-track file first
    const char* path = getBestLapPathForTrack(trackId, layoutId);
    if (loadBestLapFromFile(lap, path)) {
        return true;
    }

    // Fallback: legacy best.bin (for migration)
    if (loadBestLapFromFile(lap, getLegacyBestLapPath())) {
        ESP_LOGI(TAG, "Loaded legacy best.bin as fallback for %s/%s", trackId, layoutId);
        return true;
    }

    return false;
}

bool hasBestLap(const char* trackId, const char* layoutId) {
    if (!trackId || !layoutId) {
        return false;
    }
    if (fileExists(getBestLapPathForTrack(trackId, layoutId))) {
        return true;
    }
    // Fallback: legacy
    return fileExists(getLegacyBestLapPath());
}

uint32_t getBestLapTime(const char* trackId, const char* layoutId) {
    if (!trackId || !layoutId) {
        return UINT32_MAX;
    }

    // Try per-track file first
    const char* path = getBestLapPathForTrack(trackId, layoutId);
    if (!fileExists(path)) {
        // Fallback: legacy
        path = getLegacyBestLapPath();
        if (!fileExists(path)) {
            return UINT32_MAX;
        }
    }

    FILE* f = fopen(path, "rb");
    if (!f) {
        return UINT32_MAX;
    }

    LapHeader header;
    size_t rd = fread(&header, 1, sizeof(LapHeader), f);
    fclose(f);

    if (rd != sizeof(LapHeader) || header.magic != LAP_FILE_MAGIC) {
        return UINT32_MAX;
    }

    return header.totalTimeMs;
}

// ============================================================
// Recording Functions
// ============================================================

void startRecordingLap(uint16_t sessionId, uint16_t lapId,
                       uint8_t trackIndex, uint8_t layoutIndex) {
    s_recording.points.clear();
    s_recording.points.reserve(INIT_POINTS_RESERVE);
    s_recording.sessionId = sessionId;
    s_recording.lapId = lapId;
    s_recording.startTimestamp = (uint32_t)time(nullptr);
    s_recording.maxSpeedKmh = 0;
    s_recording.minSpeedKmh = 9999.0f;
    s_recording.totalSpeed = 0;
    s_recording.trackIndex = trackIndex;
    s_recording.layoutIndex = layoutIndex;
    s_recording.active = true;
}

void addPointToRecording(double lat, double lng, unsigned long lapTimeMs,
                         float speedKmh, float headingDeg) {
    if (!s_recording.active) {
        return;
    }

    if (s_recording.points.size() >= MAX_POINTS_PER_LAP) {
        return;  // Buffer full
    }

    StoredPoint pt;
    pt.lat = (int32_t)(lat * 1e7);
    pt.lng = (int32_t)(lng * 1e7);
    pt.lapTimeMs = lapTimeMs;
    pt.speedX10 = (uint16_t)(speedKmh * 10);
    pt.heading = (uint8_t)(headingDeg * 255.0f / 360.0f);

    s_recording.points.push_back(pt);

    if (speedKmh > s_recording.maxSpeedKmh) {
        s_recording.maxSpeedKmh = speedKmh;
    }
    if (speedKmh > 1.0f && speedKmh < s_recording.minSpeedKmh) {
        s_recording.minSpeedKmh = speedKmh;
    }
    s_recording.totalSpeed += speedKmh;
}

bool finishRecordingLap(StorableLap& outLap) {
    if (!s_recording.active || s_recording.points.empty()) {
        cancelRecording();
        return false;
    }

    outLap.points = std::move(s_recording.points);
    outLap.totalTimeMs = outLap.points.back().lapTimeMs;
    outLap.startTimestamp = s_recording.startTimestamp;
    outLap.maxSpeedKmh = s_recording.maxSpeedKmh;
    outLap.minSpeedKmh = (s_recording.minSpeedKmh < 9998.0f) ? s_recording.minSpeedKmh : 0.0f;
    outLap.avgSpeedKmh = s_recording.totalSpeed / outLap.points.size();
    outLap.sessionId = s_recording.sessionId;
    outLap.lapId = s_recording.lapId;
    outLap.trackIndex = s_recording.trackIndex;
    outLap.layoutIndex = s_recording.layoutIndex;

    s_recording = {};
    return true;
}

void cancelRecording() {
    s_recording.points.clear();
    s_recording.active = false;
}

bool isRecording() {
    return s_recording.active;
}

// ============================================================
// Listing Functions
// ============================================================

int listLaps(uint16_t sessionId, LapInfo* outList, int maxCount) {
    DIR* dir = opendir(getLapsDir());
    if (!dir) {
        return 0;
    }

    char prefix[16];
    snprintf(prefix, sizeof(prefix), "s%03u_l", sessionId);

    int count = 0;
    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr && count < maxCount) {
        const char* name = ent->d_name;
        if (strstr(name, prefix) == nullptr) {
            continue;
        }

        char path[FILENAME_BUF_SIZE];
        snprintf(path, sizeof(path), "%s/%s", getLapsDir(), name);
        FILE* f = fopen(path, "rb");
        if (!f) {
            continue;
        }

        LapHeader header;
        size_t read = fread(&header, 1, sizeof(LapHeader), f);
        fclose(f);

        if (read == sizeof(LapHeader) && header.magic == LAP_FILE_MAGIC) {
            outList[count].sessionId = header.sessionId;
            outList[count].lapId = header.lapId;
            outList[count].totalTimeMs = header.totalTimeMs;
            outList[count].pointCount = header.pointCount;
            outList[count].maxSpeedKmh = header.maxSpeedX10 / 10.0f;
            count++;
        }
    }

    closedir(dir);
    return count;
}

uint16_t getNextSessionId() {
    DIR* dir = opendir(getLapsDir());
    if (!dir) {
        return 1;
    }

    uint16_t maxSession = 0;
    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
        const char* name = ent->d_name;
        if (name[0] == 's') {
            int sid = atoi(name + 1);
            if (sid > 0 && sid <= 65534 && (uint16_t)sid > maxSession) {
                maxSession = (uint16_t)sid;
            }
        }
    }

    closedir(dir);
    if (maxSession >= 65534) {
        ESP_LOGW(TAG, "Session ID at limit (65534)");
        return 65534;
    }
    return maxSession + 1;
}

int getSessionCount() {
    DIR* dir = opendir(getLapsDir());
    if (!dir) {
        return 0;
    }

    uint16_t sessions[MAX_SESSIONS] = {};
    int sessionCount = 0;

    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
        const char* name = ent->d_name;
        if (name[0] == 's') {
            int sid = atoi(name + 1);
            bool found = false;
            for (int i = 0; i < sessionCount; i++) {
                if (sessions[i] == sid) {
                    found = true;
                    break;
                }
            }
            if (!found && sessionCount < MAX_SESSIONS) {
                sessions[sessionCount++] = sid;
            }
        }
    }

    closedir(dir);
    return sessionCount;
}

// ============================================================
// Cleanup Functions
// ============================================================

bool clearAllLaps() {
    DIR* dir = opendir(getLapsDir());
    if (!dir) {
        return false;
    }

    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
        const char* name = ent->d_name;
        if (name[0] == '.') {
            continue;
        }
        char path[FILENAME_BUF_SIZE];
        snprintf(path, sizeof(path), "%s/%s", getLapsDir(), name);
        remove(path);
    }

    closedir(dir);
    ESP_LOGI(TAG, "All laps cleared");
    return true;
}

bool clearSession(uint16_t sessionId) {
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "s%03u_", sessionId);

    DIR* dir = opendir(getLapsDir());
    if (!dir) {
        return false;
    }

    int deleted = 0;
    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
        const char* name = ent->d_name;
        if (strstr(name, prefix) != nullptr) {
            char path[FILENAME_BUF_SIZE];
            snprintf(path, sizeof(path), "%s/%s", getLapsDir(), name);
            if (remove(path) == 0) {
                deleted++;
            }
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "Deleted %d laps from session %u", deleted, sessionId);
    return deleted > 0;
}

// ============================================================
// Storage Info
// ============================================================

size_t getUsedStorage() {
    if (sdcardIsMounted()) {
        uint64_t total = sdcardGetTotalBytes();
        uint64_t free = sdcardGetFreeBytes();
        return (size_t)(total - free);
    }
    size_t total = 0;
    size_t used = 0;
    if (esp_spiffs_info(nullptr, &total, &used) != ESP_OK) {
        return 0;
    }
    return used;
}

size_t getFreeStorage() {
    if (sdcardIsMounted()) {
        return (size_t)sdcardGetFreeBytes();
    }
    size_t total = 0;
    size_t used = 0;
    if (esp_spiffs_info(nullptr, &total, &used) != ESP_OK) {
        return 0;
    }
    return total - used;
}
