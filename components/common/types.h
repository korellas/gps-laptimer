/**
 * @file types.h
 * @brief Common data structures for GPS Lap Timer
 * @version 1.0
 * 
 * This file contains all shared data structures used across
 * multiple modules. Keep this file dependency-free (no includes
 * from other project files except config.h).
 */

#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cstdint>
#include <cfloat>

// ============================================================
// GPS MODE
// ============================================================

/**
 * @brief Operating mode for GPS data source
 */
enum class GPSMode {
    GPS_HARDWARE,   // Real GPS module (u-blox)
    SIMULATION      // Simulated data (for development/testing)
};

// ============================================================
// GPS POINT
// ============================================================

/**
 * @brief Single GPS data point with timing and motion info
 */
struct GPSPoint {
    double lat = 0.0;               // Latitude (degrees)
    double lng = 0.0;               // Longitude (degrees)
    unsigned long gpsTimeMs = 0;    // GPS timestamp (ms)
    unsigned long lapTimeMs = 0;    // Lap time when recorded (ms)
    float speedKmh = 0.0f;          // Speed (km/h)
    float headingDeg = 0.0f;        // Heading/bearing (degrees, 0=North)
    bool initialized = false;       // Has this point been set with valid data

    // Check if point has valid coordinates (based on initialization flag)
    bool isValid() const {
        return initialized;
    }

    // Set coordinates and mark as valid
    void set(double latitude, double longitude) {
        lat = latitude;
        lng = longitude;
        initialized = true;
    }

    // Clear the point
    void clear() {
        lat = 0.0;
        lng = 0.0;
        gpsTimeMs = 0;
        lapTimeMs = 0;
        speedKmh = 0.0f;
        headingDeg = 0.0f;
        initialized = false;
    }
};

// ============================================================
// LAP DATA (Reference Lap)
// ============================================================

/**
 * @brief Complete lap data including GPS points and statistics
 * 
 * Used for storing reference laps for delta calculation.
 * The cumulativeDistances array is pre-calculated for efficient
 * track distance lookups during delta computation.
 */
struct LapData {
    std::vector<GPSPoint> points;               // GPS points along the lap
    std::vector<float> cumulativeDistances;     // Pre-calculated cumulative distance at each point (meters)
    float totalTrackDistance = 0.0f;            // Total track length (meters)
    unsigned long totalTimeMs = 0;              // Total lap time (ms)
    unsigned long startTimeMs = 0;              // Lap start timestamp
    float maxSpeedKmh = 0.0f;                   // Maximum speed during lap
    float avgSpeedKmh = 0.0f;                   // Average speed during lap
    
    // Clear all data
    void clear() {
        points.clear();
        cumulativeDistances.clear();
        totalTrackDistance = 0.0f;
        totalTimeMs = 0;
        startTimeMs = 0;
        maxSpeedKmh = 0.0f;
        avgSpeedKmh = 0.0f;
    }
    
    // Check if lap has valid data
    bool isValid() const {
        return !points.empty() && totalTimeMs > 0;
    }
    
    // Get number of points
    size_t size() const {
        return points.size();
    }
};

// ============================================================
// DELTA RESULT
// ============================================================

/**
 * @brief Result of delta calculation between current position and reference lap
 */
struct DeltaResult {
    float deltaSeconds = 0.0f;      // Time delta (+slower, -faster)
    float distanceMeters = FLT_MAX; // Perpendicular distance to track (meters)
    float confidence = 0.0f;        // Confidence level (0-1, based on distance)
    int refPointIndex = -1;         // Index of matched reference segment start
    float trackDistanceM = 0.0f;    // Distance along track from start (meters)
    float refTimeSec = 0.0f;        // Reference lap time at current position (seconds)
    float refSpeedKmh = 0.0f;       // Reference lap speed at current position (km/h)
    float speedDeltaKmh = 0.0f;     // Speed delta vs reference (+faster, -slower)
    bool hasSpeedDelta = false;     // Whether speed delta is valid
    
    // Check if result is valid
    bool isValid() const {
        return refPointIndex >= 0 && confidence > 0.0f;
    }
    
    // Clear/reset
    void clear() {
        deltaSeconds = 0.0f;
        distanceMeters = FLT_MAX;
        confidence = 0.0f;
        refPointIndex = -1;
        trackDistanceM = 0.0f;
        refTimeSec = 0.0f;
        refSpeedKmh = 0.0f;
        speedDeltaKmh = 0.0f;
        hasSpeedDelta = false;
    }
};

// ============================================================
// FILTERED GPS POINT
// ============================================================

/**
 * @brief GPS point with filtering metadata
 * 
 * Extends GPSPoint with information about how the point was
 * processed (filtered, interpolated, etc.)
 */
struct FilteredGPSPoint {
    GPSPoint point;                 // The GPS point data
    
    bool isValid = false;           // Is this a valid data point
    bool isInterpolated = false;    // Was position estimated (dead reckoning)
    bool wasSpikeFiltered = false;  // Was a spike detected and filtered
    
    double rawLat = 0.0;            // Original latitude before filtering
    double rawLng = 0.0;            // Original longitude before filtering
    float confidence = 1.0f;        // Position confidence (0-1)
    
    // Clear all
    void clear() {
        point.clear();
        isValid = false;
        isInterpolated = false;
        wasSpikeFiltered = false;
        rawLat = 0.0;
        rawLng = 0.0;
        confidence = 1.0f;
    }
};

// ============================================================
// SECTOR TIME
// ============================================================

/**
 * @brief Timing data for a single sector
 */
struct SectorTime {
    int sectorIndex = -1;               // Sector index (0-based)
    unsigned long timeMs = 0;           // Time for this sector (ms)
    unsigned long cumulativeMs = 0;     // Cumulative time from lap start (ms)
    bool isComplete = false;            // Has this sector been completed
    
    void clear() {
        sectorIndex = -1;
        timeMs = 0;
        cumulativeMs = 0;
        isComplete = false;
    }
};

/**
 * @brief Delta information for a sector
 */
struct SectorDelta {
    int sectorIndex = -1;           // Sector index (0-based)
    float deltaSeconds = 0.0f;      // Delta vs reference (+slower, -faster)
    bool isComplete = false;        // Is this a final sector time (vs in-progress)
    
    void clear() {
        sectorIndex = -1;
        deltaSeconds = 0.0f;
        isComplete = false;
    }
};

// ============================================================
// DELTA DISPLAY MODE
// ============================================================

/**
 * @brief How to display delta time
 */
enum class DeltaDisplayMode {
    LAP_TOTAL,          // Show delta for entire lap (default)
    CURRENT_SECTOR,     // Show delta for current sector in progress
    LAST_SECTOR         // Show delta for most recently completed sector
};

// ============================================================
// APP STATE
// ============================================================

/**
 * @brief Application state machine states
 */
enum class AppState {
    BOOT,               // System booting (splash screen)
    GPS_SEARCHING,      // Waiting for GPS fix
    TRACK_DETECTING,    // GPS acquired, detecting which track
    TRACK_CONFIRM,      // Track detected, awaiting user confirmation
    LAP_READY,          // Ready to start lap (waiting at finish line)
    LAP_RUNNING,        // Lap in progress
    LAP_COMPLETE,       // Lap just finished (showing result briefly)
    SETTINGS,           // Settings menu
    ERROR               // Error state
};

/**
 * @brief Events that can trigger state transitions
 */
enum class AppEvent {
    BOOT_COMPLETE,      // Boot sequence finished
    GPS_ACQUIRED,       // Got GPS fix
    GPS_LOST,           // Lost GPS signal
    TRACK_DETECTED,     // Auto-detected a track
    TRACK_CONFIRMED,    // User confirmed track selection
    TRACK_CHANGED,      // User changed track/layout
    NO_TRACK_FOUND,     // Could not detect any known track
    FINISH_CROSSED,     // Crossed finish line
    LAP_DISPLAY_DONE,   // Finished showing lap complete screen
    TIMEOUT,            // Generic timeout
    USER_RESET,         // User requested reset
    ERROR_OCCURRED,     // An error occurred
    ENTER_SETTINGS,     // User entered settings
    EXIT_SETTINGS       // User exited settings
};

// ============================================================
// OTA STATE
// ============================================================

/**
 * @brief OTA 업데이트 상태
 */
enum class OTAState {
    IDLE,           // OTA 미진행
    RECEIVING,      // 데이터 수신 중
    VALIDATING,     // 이미지 검증 중
    COMPLETE,       // 완료, 리부트 대기
    ERROR           // 오류 발생
};

// ============================================================
// IMU DATA
// ============================================================

/**
 * @brief IMU 6축 데이터 (가속도 + 자이로)
 */
struct ImuData {
    float accelX = 0.0f;       // 캘리브 후 가속도 X (g)
    float accelY = 0.0f;       // 캘리브 후 가속도 Y (g)
    float accelZ = 0.0f;       // 캘리브 후 가속도 Z (g)
    float rawAccelX = 0.0f;    // 원시 가속도 X (g)
    float rawAccelY = 0.0f;    // 원시 가속도 Y (g)
    float rawAccelZ = 0.0f;    // 원시 가속도 Z (g)
    float gyroX = 0.0f;        // 자이로 X (°/s)
    float gyroY = 0.0f;        // 자이로 Y (°/s)
    float gyroZ = 0.0f;        // 자이로 Z (°/s)
    float temperature = 0.0f;  // 칩 온도 (°C)
    bool valid = false;
    uint32_t timestampMs = 0;
};

/**
 * @brief IMU 캘리브레이션 오프셋
 */
struct ImuCalibration {
    float accelOffsetX = 0.0f;
    float accelOffsetY = 0.0f;
    float accelOffsetZ = 0.0f;
    float gyroOffsetX = 0.0f;
    float gyroOffsetY = 0.0f;
    float gyroOffsetZ = 0.0f;
    bool calibrated = false;
    int samplesUsed = 0;
};

// ============================================================
// APP CONTEXT - Consolidated application state
// ============================================================

/**
 * @brief Consolidated application state
 *
 * Replaces individual global variables that were shared via extern.
 * Single instance `gApp` is defined in main.cpp.
 */
struct AppContext {
    // GPS mode
    GPSMode currentGpsMode = GPSMode::GPS_HARDWARE;

    // Reference lap data
    LapData referenceLap;

    // Current GPS position
    GPSPoint currentPoint;
    GPSPoint previousPoint;

    // Current delta calculation result
    DeltaResult currentDelta;

    // Current lap/session tracking
    uint16_t currentLapNumber = 1;
    uint16_t currentSessionNumber = 1;

    // Best lap tracking
    uint32_t bestLapTimeMs = UINT32_MAX;
    bool hasValidReferenceLap = false;

    // Segment search continuity index
    int lastValidSegmentIndex = -1;

    // Top 3 laps for display (updated only on lap completion)
    struct Top3Lap {
        uint16_t lapNumber = 0;
        uint32_t lapTimeMs = UINT32_MAX;
    };
    Top3Lap top3Laps[3];

    // Battery monitoring
    float batteryVoltage = 0.0f;   // 배터리 전압 (V)
    float batteryPercent = -1.0f;  // 배터리 잔량 (%), -1 = 미측정

    // GPS 시간 동기화
    bool gpsTimeSet = false;      // GPS에서 시스템 시계 설정 완료 여부

    // SD 카드 상태
    bool sdCardMounted = false;   // SD 카드 마운트 여부

    // IMU (QMI8658C)
    ImuData imuData;
    ImuCalibration imuCalibration;
    bool imuReady = false;

    // Sensor Fusion (GPS+IMU)
    float fusedSpeedKmh = 0.0f;      // 칼만 필터 출력 속도 (km/h)
    bool  fusionActive = false;       // 퓨전 동작 중 (칼만 필터 초기화됨)
    bool  fusionInDR = false;         // GPS 없이 predict만 진행 중
    float lastGpsHeadingRad = 0.0f;   // 마지막 GPS heading (forward 투영용, rad)
    uint32_t fusionCalibDoneMs = 0;   // 캘리브레이션 완료 시각 (UI 알림용)

    // WiFi Portal settings (SPIFFS에 저장)
    char phoneNumber[32] = "";    // 전화번호 플레이트

    // OTA 상태
    OTAState otaState = OTAState::IDLE;
    float otaProgress = 0.0f;          // 0.0 - 1.0
    uint32_t otaReceivedBytes = 0;
    uint32_t otaTotalBytes = 0;
    char otaErrorMsg[64] = "";

    // Helper: check if in simulation mode
    bool isSimulationMode() const {
        return currentGpsMode == GPSMode::SIMULATION;
    }
};

// Global application state instance (defined in components/common/app_context.cpp)
extern AppContext gApp;

#endif // TYPES_H
