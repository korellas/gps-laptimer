/**
 * @file config.h
 * @brief Central configuration for GPS Lap Timer
 * @version 1.0
 * 
 * All hardware pins, timing constants, and tunable parameters
 * are defined here for easy modification.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// ============================================================
// DEBUG SETTINGS
// ============================================================

// Runtime-toggleable debug output (use 'v' serial command)
extern bool g_debugOutput;
#define DEBUG_OUTPUT g_debugOutput

constexpr unsigned long DEBUG_INTERVAL_MS = 5000;

// ============================================================
// HARDWARE PINS - Waveshare ESP32-S3-Touch-LCD-3.49
// ============================================================

// GPS Module
constexpr int GPS_RX_PIN     = 44;  // GPIO44 (RXD) - Connect to GPS TX (파랑)
constexpr int GPS_TX_PIN     = 43;  // GPIO43 (TXD) - Connect to GPS RX (초록)
constexpr int GPS_PPS_PIN    = 1;   // GPIO1 - PPS 타임펄스 입력 (하양)
constexpr int GPS_ENABLE_PIN = 0;   // GPIO0 - 모듈 ENABLE/EXTINT (노랑)

// PWR Button (배터리 모드 전원 제어)
constexpr int PWR_BUTTON_PIN = 16;  // GPIO16 - PWR button (active low)
constexpr unsigned long POWER_OFF_HOLD_MS = 2000;  // 2초 길게 누르면 전원 오프

// Battery ADC (GPIO4 = ADC1_CHANNEL_3, 전압 분배기 3:1)
constexpr float BATTERY_DIVIDER_FACTOR = 3.0f;
constexpr unsigned long BATTERY_READ_INTERVAL_MS = 2000;  // 2초마다 측정

// GPS UART Settings — baud rate 상수는 ublox_gps.h에 정의
// (UBLOX_BAUD_INIT = 9600, UBLOX_BAUD_TARGET = 115200)

// ============================================================
// UPDATE INTERVALS
// ============================================================

// Simulation update rate (20Hz)
constexpr unsigned long SIM_UPDATE_INTERVAL_MS = 50;

// Display refresh rate (60Hz for smoother animation)
constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 16;

// GPS timeout - consider signal lost after this duration
constexpr unsigned long GPS_TIMEOUT_MS = 3000;

// ============================================================
// DELTA CALCULATION PARAMETERS
// ============================================================

// Maximum perpendicular distance from track to consider valid (meters)
constexpr float MAX_PROJECTION_DISTANCE_M = 50.0f;

// Confidence decay factor for delta calculation
// Higher = faster decay when far from track
constexpr float CONFIDENCE_DECAY_FACTOR = 3.0f;

// Segment search window for continuity (number of segments)
// Prevents jumping to wrong part of track when it crosses itself
constexpr int SEGMENT_SEARCH_WINDOW = 50;

// Minimum segment search backward (for slight backtracking)
constexpr int SEGMENT_SEARCH_BACK = 10;

// ============================================================
// LAP TIMING
// ============================================================

// Minimum lap time to consider valid (prevents false triggers)
constexpr unsigned long MIN_LAP_TIME_MS = 30000;  // 30 seconds

// Delta suppression after lap reset (prevents jumps)
constexpr unsigned long DELTA_SUPPRESS_AFTER_RESET_MS = 2000;

// Lap complete display duration
constexpr unsigned long LAP_COMPLETE_DISPLAY_MS = 5000;

// ============================================================
// GPS FILTER SETTINGS
// ============================================================

// Spike detection - maximum allowed jump distance (meters)
// If GPS position jumps more than this in one update, it's a spike
constexpr float GPS_MAX_JUMP_DISTANCE_M = 50.0f;

// Spike detection - speed-based threshold multiplier
// Jump distance must be less than: speed * time * this multiplier
constexpr float GPS_JUMP_SPEED_MULTIPLIER = 2.0f;

// Enable position smoothing (moving average)
constexpr bool GPS_ENABLE_SMOOTHING = true;

// Smoothing window size (number of points to average)
constexpr int GPS_SMOOTHING_WINDOW = 3;

// ============================================================
// DEAD RECKONING SETTINGS
// ============================================================

// Enable dead reckoning when GPS signal is lost
constexpr bool ENABLE_DEAD_RECKONING = true;

// Maximum duration to use dead reckoning (ms)
// After this, stop estimating and wait for GPS
constexpr unsigned long MAX_DEAD_RECKONING_MS = 5000;

// Minimum speed to use dead reckoning (km/h)
// Below this speed, position estimation is unreliable
constexpr float MIN_DEAD_RECKONING_SPEED_KMH = 10.0f;

// ============================================================
// SIMULATION SETTINGS
// ============================================================

// Default satellite count to display in simulation mode
constexpr uint8_t SIM_SATELLITE_COUNT = 10;

// ============================================================
// TRACK DETECTION
// ============================================================

// How often to check for track detection when not on a track (ms)
constexpr unsigned long TRACK_DETECTION_INTERVAL_MS = 1000;

// Minimum GPS samples before attempting track detection
constexpr int MIN_SAMPLES_FOR_TRACK_DETECTION = 5;

// ============================================================
// SECTOR TIMING
// ============================================================

// Maximum number of sectors per layout
constexpr int MAX_SECTORS_PER_LAYOUT = 8;

// Sector delta display duration after sector completion (ms)
constexpr unsigned long SECTOR_DELTA_DISPLAY_MS = 5000;

// Maximum sector progress ratio (clamp for slow laps, 2.0 = allow up to 2x ref time)
constexpr float SECTOR_PROGRESS_MAX = 2.0f;

// ============================================================
// STORAGE
// ============================================================

// Maximum laps to store per session
constexpr int MAX_LAPS_PER_SESSION = 100;

// Maximum sessions to keep
constexpr int MAX_SESSIONS = 50;

// Maximum GPS points per reference/simulation lap
// (Everland ref = 1514, sim laps = ~1360)
constexpr int MAX_REFERENCE_POINTS = 1600;

// ============================================================
// DISPLAY
// ============================================================

// Landscape rotation for the Waveshare panel.
// 90 = default landscape, 270 = landscape flipped by 180 degrees.
constexpr int DISPLAY_ROTATION_DEG = 270;

// Delta bar range in seconds (±)
constexpr float DELTA_BAR_RANGE_SECONDS = 2.0f;

// Notification default duration
constexpr unsigned long NOTIFICATION_DEFAULT_MS = 2000;

// Gesture swipe distance threshold (Manhattan distance)
constexpr int TOUCH_SWIPE_THRESHOLD_PX = 30;

// ============================================================
// STARTUP SCREEN
// ============================================================

// Application version string
// 빌드 시 git tag에서 자동 주입: -DAPP_VERSION_FROM_GIT="v0.5.0"
// 태그 없으면 git describe (예: v0.4.0-3-gabcdef) 또는 기본값 사용
#ifdef APP_VERSION_FROM_GIT
constexpr const char* APP_VERSION = APP_VERSION_FROM_GIT;
#else
constexpr const char* APP_VERSION = "v0.9.0-dev";
#endif

// 시뮬레이션 모드 시작 화면 표시 시간 (ms)
constexpr unsigned long STARTUP_SIM_DISPLAY_MS = 2000;

// GPS 모드 하드 타임아웃 - 이후 터치/키로 dismiss 가능 (ms)
constexpr unsigned long STARTUP_GPS_TIMEOUT_MS = 60000;

// 전환 애니메이션 시간 (ms)
constexpr unsigned long STARTUP_TRANSITION_MS = 300;

// ============================================================
// TASK SETTINGS
// ============================================================

// Main task stack size (bytes) - needs to be large for C++ STL
constexpr int MAIN_TASK_STACK_SIZE = 8192;

// Main task priority (1-24, higher = more priority)
constexpr int MAIN_TASK_PRIORITY = 5;

// Core to run main task on (0 or 1)
constexpr int MAIN_TASK_CORE = 1;

#endif // CONFIG_H
