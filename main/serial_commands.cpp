/**
 * @file serial_commands.cpp
 * @brief Serial command handler implementation for GPS Lap Timer (ESP-IDF)
 * @version 2.0 - ESP-IDF native implementation
 *
 * Handles all serial commands for configuration, debugging, and control.
 * Uses ESP-IDF UART (stdin) for serial communication.
 */

#include "serial_commands.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "config.h"
#include "types.h"
#include "waveshare_display.h"
#include "finish_line.h"
#include "lap_storage.h"
#include "protocol.hpp"
#include "simulation.h"
#include "gps_processor.h"
#include "wifi_portal.h"
#include "ublox_gps.h"
#include "sdcard_manager.h"
#include "ble_ota.h"
#include "qmi8658c.h"

static const char *TAG = "SERIAL_CMD";

// ============================================================
// EXTERNAL REFERENCES
// ============================================================

// Helper functions to access sim state from main.cpp
int getSimCurrentLapIdx(void);
int getSimCurrentPointIdx(void);

// ============================================================
// MODULE STATE
// ============================================================

// Command echo setting
static bool s_echoEnabled = false;

// UART read buffer
static uint8_t s_rxBuffer[64];
static bool s_initialized = false;
static bool s_stdio_ready = false;

// ============================================================
// SERIAL HELPER FUNCTIONS
// ============================================================

static void serial_printf(const char* format, ...) {
    if (!s_stdio_ready) {
        return;
    }

    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    fputs(buf, stdout);
    fflush(stdout);
}

static char serial_read(void) {
    if (!s_stdio_ready) {
        return 0;
    }

    ssize_t n = read(STDIN_FILENO, s_rxBuffer, 1);
    if (n == 1) {
        return (char)s_rxBuffer[0];
    }
    // n == 0: no data, n < 0: error (EAGAIN in non-blocking mode is normal)
    return 0;
}

// ============================================================
// COMMAND HANDLERS
// ============================================================

/**
 * @brief Handle mode toggle command (e/E)
 */
static void cmdToggleMode(void) {
    gApp.currentGpsMode = (gApp.currentGpsMode == GPSMode::GPS_HARDWARE)
                          ? GPSMode::SIMULATION
                          : GPSMode::GPS_HARDWARE;

    if (gApp.isSimulationMode()) {
        serial_printf("Switched to EMULATION mode\n");
        resetSimulation();
    } else {
        serial_printf("Switched to LAPTIMER mode\n");
        resetRealGPS();
    }
}

/**
 * @brief Handle reset command (r/R)
 */
static void cmdReset(void) {
    if (gApp.isSimulationMode()) {
        resetSimulation();
    } else {
        resetRealGPS();
        resetDeltaHistory();
        serial_printf("Lap reset\n");
    }
}

/**
 * @brief Handle finish line set command (f/F)
 */
static void cmdSetFinishLine(void) {
    if (gApp.currentPoint.lat != 0 && gApp.currentPoint.lng != 0) {
        if (setFinishLineFromCurrentPos(gApp.currentPoint.lat, gApp.currentPoint.lng,
                                         gApp.currentPoint.headingDeg)) {
            showNotification("FINISH SET", 2000);
            serial_printf("Finish line set at current position\n");
        }
    } else {
        serial_printf("No GPS position available\n");
    }
}

/**
 * @brief Handle clear finish line command (c/C)
 */
static void cmdClearFinishLine(void) {
    clearFinishLine();
    showNotification("LINE CLEARED", 1500);
    serial_printf("Finish line cleared\n");
}

/**
 * @brief Handle status command (s/S)
 */
static void cmdShowStatus(void) {
    serial_printf("\n=== STATUS ===\n");
    serial_printf("Mode: %s\n", gApp.isSimulationMode() ? "EMULATION" : "LAPTIMER");
    serial_printf("Session: %u, Lap: %u\n", gApp.currentSessionNumber, gApp.currentLapNumber);
    serial_printf("Reference: %d pts, %.2fs\n",
        (int)gApp.referenceLap.points.size(), gApp.referenceLap.totalTimeMs / 1000.0f);
    serial_printf("Best lap: %.2fs\n", gApp.bestLapTimeMs / 1000.0f);
    serial_printf("Finish line: %s\n", isFinishLineConfigured() ? "SET" : "NOT SET");

    if (gApp.isSimulationMode()) {
        int lapIdx = getSimCurrentLapIdx();
        int pointIdx = getSimCurrentPointIdx();
        serial_printf("Progress: lap data %d, point %d\n", lapIdx, pointIdx);
    }

    serial_printf("Position: %.6f, %.6f\n", gApp.currentPoint.lat, gApp.currentPoint.lng);
    serial_printf("Delta: %.2fs (dist: %.1fm)\n",
        gApp.currentDelta.deltaSeconds, gApp.currentDelta.distanceMeters);
    serial_printf("Storage: %u/%u bytes\n",
        (unsigned int)getUsedStorage(),
        (unsigned int)(getUsedStorage() + getFreeStorage()));
    serial_printf("Heap: %d bytes\n\n", (int)esp_get_free_heap_size());
}

/**
 * @brief Handle list laps command (l/L)
 */
static void cmdListLaps(void) {
    LapInfo laps[10];
    int count = listLaps(gApp.currentSessionNumber, laps, 10);

    serial_printf("\n=== Session %u Laps ===\n", gApp.currentSessionNumber);
    for (int i = 0; i < count; i++) {
        serial_printf("Lap %u: %.2fs (%u pts)\n",
            laps[i].lapId, laps[i].totalTimeMs / 1000.0f, laps[i].pointCount);
    }

    if (count == 0) {
        serial_printf("No laps saved\n");
    }
    serial_printf("\n");
}

/**
 * @brief Handle new session command (n/N)
 */
static void cmdNewSession(void) {
    gApp.currentSessionNumber = getNextSessionId();
    gApp.currentLapNumber = 1;
    serial_printf("New session: %u\n", gApp.currentSessionNumber);
    showNotification("NEW SESSION", 1500);
}

/**
 * @brief Handle display test command (d/D)
 */
static void cmdDisplayTest(void) {
    displayTest();
}

/**
 * @brief Handle verbose toggle command (v/V)
 */
static void cmdToggleVerbose(void) {
    extern bool g_debugOutput;
    g_debugOutput = !g_debugOutput;
    serial_printf("Debug output: %s\n", g_debugOutput ? "ON" : "OFF");
}

/**
 * @brief Handle GPS diagnostics command (g/G)
 */
static void cmdGPSDiagnostics(void) {
    UBloxData ubx = getUBloxData();
    UBloxStats stats = getUBloxStats();
    const GPSProcessorState& gs = getGPSProcessorState();

    uint32_t totalMsgs = stats.checksumOk + stats.checksumFail;
    int healthPct = totalMsgs > 0 ? (int)(stats.checksumOk * 100 / totalMsgs) : 0;

    serial_printf("\n=== GPS DIAGNOSTICS ===\n");
    serial_printf("Fix type: %d (%s)\n", ubx.fixType,
        ubx.fixType == 0 ? "No fix" :
        ubx.fixType == 2 ? "2D" :
        ubx.fixType == 3 ? "3D" :
        ubx.fixType == 4 ? "GNSS+DR" : "Other");
    serial_printf("Satellites: %d\n", ubx.satellites);
    serial_printf("Position: %.7f, %.7f\n", ubx.lat, ubx.lon);
    serial_printf("Altitude: %.1f m\n", ubx.altitudeM);
    serial_printf("Speed: %.1f km/h\n", ubx.speedKmh);
    serial_printf("Heading: %.1f deg\n", ubx.headingDeg);
    serial_printf("HDOP: %.1f\n", ubx.hdop);
    serial_printf("Valid: %s\n", ubx.valid ? "YES" : "NO");
    serial_printf("iTOW: %u ms\n", ubx.iTOW);

    if (ubx.timeValid) {
        serial_printf("UTC: %04u-%02u-%02u %02u:%02u:%02u\n",
            ubx.year, ubx.month, ubx.day,
            ubx.hour, ubx.minute, ubx.second);
    }

    serial_printf("Rate: %.1f Hz\n", stats.measuredHz);
    serial_printf("UART: %lu bytes, OK=%lu ERR=%lu (%d%%)\n",
        (unsigned long)stats.totalBytes,
        (unsigned long)stats.checksumOk,
        (unsigned long)stats.checksumFail,
        healthPct);
    serial_printf("Baud: %d\n", UBLOX_BAUD_TARGET);
    serial_printf("Lap started: %s\n", gs.lapStarted ? "YES" : "NO");
    serial_printf("Signal lost: %s\n", gs.gpsSignalLost ? "YES" : "NO");
    serial_printf("\n");
}

/**
 * @brief Handle IMU diagnostics command (i/I)
 */
static void cmdImuDiagnostics(void) {
    if (!imuIsReady()) {
        serial_printf("IMU: NOT INITIALIZED\n");
        return;
    }

    serial_printf("\n=== IMU DIAGNOSTICS (QMI8658C) ===\n");

    ImuData data = gApp.imuData;
    ImuCalibration cal = gApp.imuCalibration;

    serial_printf("Accel (calibrated): X=%+.3f Y=%+.3f Z=%+.3f g\n",
        data.accelX, data.accelY, data.accelZ);
    serial_printf("Accel (raw):        X=%+.3f Y=%+.3f Z=%+.3f g\n",
        data.rawAccelX, data.rawAccelY, data.rawAccelZ);
    serial_printf("Gyro:               X=%+.2f Y=%+.2f Z=%+.2f dps\n",
        data.gyroX, data.gyroY, data.gyroZ);

    float gMag = sqrtf(data.accelX * data.accelX +
                       data.accelY * data.accelY +
                       data.accelZ * data.accelZ);
    serial_printf("G magnitude: %.3f g\n", gMag);
    serial_printf("Temperature: %.1f C\n", data.temperature);
    serial_printf("Valid: %s, Age: %lu ms\n",
        data.valid ? "YES" : "NO",
        (unsigned long)((esp_timer_get_time() / 1000) - data.timestampMs));

    serial_printf("Calibration: %s (%d samples)\n",
        cal.calibrated ? "YES" : "NO", cal.samplesUsed);
    if (cal.calibrated) {
        serial_printf("  Accel offset: X=%+.4f Y=%+.4f Z=%+.4f g\n",
            cal.accelOffsetX, cal.accelOffsetY, cal.accelOffsetZ);
        serial_printf("  Gyro offset:  X=%+.2f Y=%+.2f Z=%+.2f dps\n",
            cal.gyroOffsetX, cal.gyroOffsetY, cal.gyroOffsetZ);
    }
    serial_printf("\n");
}

/**
 * @brief Handle help command (h/H/?)
 */
static void cmdShowHelp(void) {
    serial_printf("\n=== COMMANDS ===\n");
    serial_printf("e - Toggle GPS/simulation mode\n");
    serial_printf("r - Reset current lap\n");
    serial_printf("f - Set finish line at current position\n");
    serial_printf("c - Clear finish line\n");
    serial_printf("s - Show status\n");
    serial_printf("g - GPS diagnostics\n");
    serial_printf("i - IMU diagnostics (QMI8658C)\n");
    serial_printf("l - List saved laps\n");
    serial_printf("n - Start new session\n");
    serial_printf("d - Display test\n");
    serial_printf("v - Toggle verbose debug output\n");
    serial_printf("w - Toggle WiFi on/off\n");
    serial_printf("b - Toggle BLE OTA mode\n");
    serial_printf("m - SD card status\n");
    serial_printf("p - Power off (battery mode)\n");
    serial_printf("h - Show this help\n\n");
}

// ============================================================
// PUBLIC API IMPLEMENTATION
// ============================================================

void initSerialCommands(void) {
    // Important: GPIO43/44 are used by GPS (UART2), so don't touch UART0 here.
    // Use USB Serial JTAG console (auto-configured by sdkconfig) via VFS stdin/stdout.

    // Small delay to let console settle after boot
    vTaskDelay(pdMS_TO_TICKS(200));

    // Set stdin to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    // Drain any pending input (monitor responsiveness test data)
    char dummy[128];
    int drained = 0;
    while (true) {
        ssize_t n = read(STDIN_FILENO, dummy, sizeof(dummy));
        if (n <= 0) break;
        drained += n;
    }

    if (drained > 0) {
        ESP_LOGI(TAG, "Drained %d bytes from stdin buffer", drained);
    }

    s_stdio_ready = true;
    s_initialized = true;
    s_echoEnabled = false;

    // Send startup message to confirm console is active
    printf("\n");
    printf("========================================\n");
    printf("  GPS Lap Timer Console Ready\n");
    printf("  Press 'h' for help\n");
    printf("========================================\n");
    fflush(stdout);

    ESP_LOGI(TAG, "Serial console ready");
}

void handleSerialCommands(void) {
    if (!s_initialized) {
        return;
    }

    char cmd = serial_read();
    if (cmd == 0) {
        return;
    }

    // 시작 화면 중 아무 키 → dismiss
    if (isStartupScreenActive()) {
        dismissStartupScreen();
        return;
    }

    switch (cmd) {
        case 'e':
        case 'E':
            cmdToggleMode();
            break;

        case 'r':
        case 'R':
            cmdReset();
            break;

        case 'f':
        case 'F':
            cmdSetFinishLine();
            break;

        case 'c':
        case 'C':
            cmdClearFinishLine();
            break;

        case 's':
        case 'S':
            cmdShowStatus();
            break;

        case 'l':
        case 'L':
            cmdListLaps();
            break;

        case 'n':
        case 'N':
            cmdNewSession();
            break;

        case 'd':
        case 'D':
            cmdDisplayTest();
            break;

        case 'g':
        case 'G':
            cmdGPSDiagnostics();
            break;

        case 'i':
        case 'I':
            cmdImuDiagnostics();
            break;

        case 'v':
        case 'V':
            cmdToggleVerbose();
            break;

        case 'w':
        case 'W':
            toggleWifiPortal();
            serial_printf("WiFi: %s\n", isWifiPortalActive() ? "ON" : "OFF");
            break;

        case 'b':
        case 'B':
            if (isBleOtaActive()) {
                esp_err_t err = stopBleOta();
                serial_printf("BLE OTA: %s\n", err == ESP_OK ? "OFF" : esp_err_to_name(err));
            } else {
                esp_err_t err = startBleOta();
                if (err == ESP_OK) {
                    serial_printf("BLE OTA: ON (LAPTIMER-OTA)\n");
                } else {
                    serial_printf("BLE OTA start failed: %s\n", esp_err_to_name(err));
                }
            }
            break;

        case 'm':
        case 'M':
            if (sdcardIsMounted()) {
                uint64_t total = sdcardGetTotalBytes();
                uint64_t free = sdcardGetFreeBytes();
                serial_printf("SD Card: MOUNTED\n");
                serial_printf("  Total: %.1f MB\n", total / (1024.0 * 1024.0));
                serial_printf("  Free:  %.1f MB\n", free / (1024.0 * 1024.0));
                serial_printf("  Laps:  /sdcard/laps/\n");
            } else {
                serial_printf("SD Card: NOT MOUNTED\n");
            }
            break;

        case 'p':
        case 'P':
            serial_printf("Power off...\n");
            systemPowerOff();
            break;

        case 'h':
        case 'H':
        case '?':
            cmdShowHelp();
            break;

        // Ignore whitespace
        case '\r':
        case '\n':
        case ' ':
        case '\t':
            break;

        default:
            if (cmd >= ' ' && cmd <= '~') {
                serial_printf("Unknown command: '%c'. Press 'h' for help.\n", cmd);
            }
            break;
    }
}

void setCommandEcho(bool enabled) {
    s_echoEnabled = enabled;
}

bool isCommandEchoEnabled(void) {
    return s_echoEnabled;
}
