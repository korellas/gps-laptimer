/**
 * @file main.cpp
 * @brief GPS Lap Timer for Waveshare ESP32-S3-Touch-LCD-3.49 (ESP-IDF)
 * @version 8.0 - ESP-IDF native implementation
 *
 * This is the main entry point using ESP-IDF's app_main().
 * All functionality is delegated to modules:
 * - modes/simulation.cpp - Simulation mode
 * - modes/gps_processor.cpp - GPS hardware mode
 * - serial_commands.cpp - Serial CLI
 * - waveshare_display.cpp - Display driver (LVGL)
 */

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"

#include "config.h"
#include "types.h"
#include "waveshare_display.h"
#include "geo_utils.h"
#include "finish_line.h"
#include "lap_storage.h"
#include "ublox_gps.h"
#include "serial_commands.h"
#include "simulation.h"
#include "gps_processor.h"
#include "protocol.hpp"
#include "delta_calculator.h"
#include "sector_timing.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "rom/ets_sys.h"  // ets_delay_us()
#include "wifi_portal.h"
#include "esp_pm.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "ota_manager.h"
#include <sys/time.h>      // settimeofday()
#include <time.h>          // time(), localtime()
#include "pcf85063.h"
#include "qmi8658c.h"
#include "sdcard_manager.h"

static const char *TAG = "MAIN";

// 내부 RAM 진단 매크로
#define LOG_INTRAM(label) \
    ESP_LOGI(TAG, "INTRAM[%-16s] free=%6lu largest=%6lu", label, \
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), \
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT))

// Runtime debug output flag (default: off for clean console)
bool g_debugOutput = false;

// ============================================================
// HELPER FUNCTIONS
// ============================================================

// ESP-IDF compatible millis()
static inline uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static constexpr time_t MIN_VALID_UTC_EPOCH = 1704067200;  // 2024-01-01 00:00:00 UTC
static constexpr unsigned long RTC_RETRY_INTERVAL_MS = 5000;

static bool isSystemClockValidUtc(time_t nowUtc) {
    return nowUtc >= MIN_VALID_UTC_EPOCH;
}

static bool trySyncSystemClockFromRTC(void) {
    if (!rtcIsReady()) return false;

    struct tm rtcTime = {};
    if (!rtcRead(&rtcTime) || rtcTime.tm_year < (2024 - 1900)) {
        return false;
    }

    time_t utcEpoch = mktime(&rtcTime);
    struct timeval tv = { .tv_sec = utcEpoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    gApp.gpsTimeSet = true;

    ESP_LOGI(TAG, "System clock refreshed from RTC: %04d-%02d-%02d %02d:%02d:%02d UTC",
             rtcTime.tm_year + 1900, rtcTime.tm_mon + 1, rtcTime.tm_mday,
             rtcTime.tm_hour, rtcTime.tm_min, rtcTime.tm_sec);
    return true;
}

static bool readValidRtcUtc(struct tm* outUtc) {
    if (!outUtc || !rtcIsReady()) return false;

    struct tm rtcTime = {};
    if (!rtcRead(&rtcTime) || rtcTime.tm_year < (2024 - 1900)) {
        return false;
    }

    *outUtc = rtcTime;
    return true;
}

// ============================================================
// BATTERY ADC
// ============================================================

static adc_oneshot_unit_handle_t s_adcHandle = NULL;
static adc_cali_handle_t s_adcCaliHandle = NULL;
static bool s_adcCaliEnabled = false;

// 저전압 자동 셧다운 (3.5V 연속 3회 도달 시)
static constexpr float BATTERY_SHUTDOWN_VOLTAGE = 3.5f;
static constexpr int BATTERY_SHUTDOWN_COUNT = 3;
static int s_lowVoltageCount = 0;

// 전압(V) → SoC% 변환 (G6EJD 4차 다항식 기반)
// Source: G6EJD polynomial for LiPo discharge curve
static float voltageToPercent(float voltage)
{
    // Clamp at voltage limits
    if (voltage >= 4.2f) return 100.0f;
    if (voltage <= 3.524f) return 0.0f;

    // 4th-order polynomial: percentage = 2808.3808*v⁴ - 43560.9157*v³ + 252848.5888*v² - 650767.4615*v + 626532.5703
    float v = voltage;
    float v2 = v * v;
    float v3 = v2 * v;
    float v4 = v3 * v;

    float percentage = 2808.3808f * v4
                     - 43560.9157f * v3
                     + 252848.5888f * v2
                     - 650767.4615f * v
                     + 626532.5703f;

    // Clamp result to valid range
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    return percentage;
}

static void initBatteryADC(void)
{
    adc_oneshot_unit_init_cfg_t unitCfg = { .unit_id = ADC_UNIT_1, .clk_src = ADC_RTC_CLK_SRC_DEFAULT, .ulp_mode = ADC_ULP_MODE_DISABLE };
    if (adc_oneshot_new_unit(&unitCfg, &s_adcHandle) != ESP_OK) {
        ESP_LOGE(TAG, "Battery ADC: unit init failed");
        return;
    }

    adc_oneshot_chan_cfg_t chanCfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(s_adcHandle, ADC_CHANNEL_3, &chanCfg);

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t caliCfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_3,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&caliCfg, &s_adcCaliHandle) == ESP_OK) {
        s_adcCaliEnabled = true;
    }
#endif

    ESP_LOGI(TAG, "Battery ADC: OK (GPIO4, cali=%s)", s_adcCaliEnabled ? "yes" : "no");
}

// 배터리 ADC 측정 (main loop에서 60초마다 호출, ~2.6ms 소요)
static void readBattery(void)
{
    if (!s_adcHandle) return;

    static constexpr int ADC_TOTAL_READS = 13;  // 1 dummy + 12 samples
    static constexpr int ADC_SAMPLES = 12;
    static constexpr int ADC_TRIM = 4;           // 상하 각 4개 버림 → 중앙 4개 평균

    int allRaw[ADC_TOTAL_READS];
    int readCount = 0;

    for (int i = 0; i < ADC_TOTAL_READS; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adcHandle, ADC_CHANNEL_3, &raw) == ESP_OK) {
            allRaw[readCount++] = raw;
        }
        if (i < ADC_TOTAL_READS - 1) {
            ets_delay_us(200);
        }
    }

    if (readCount < 7) return;

    int samples[ADC_SAMPLES];
    int validCount = (readCount - 1 > ADC_SAMPLES) ? ADC_SAMPLES : (readCount - 1);
    for (int i = 0; i < validCount; i++) {
        samples[i] = allRaw[i + 1];  // allRaw[0]은 dummy read, 버림
    }

    // 삽입 정렬
    for (int i = 1; i < validCount; i++) {
        int key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    // 상하 trim 후 중앙 평균
    int trimLow = ADC_TRIM;
    int trimHigh = validCount - ADC_TRIM;
    if (trimHigh <= trimLow) { trimLow = 0; trimHigh = validCount; }

    int sum = 0;
    for (int i = trimLow; i < trimHigh; i++) {
        sum += samples[i];
    }
    float avgRaw = (float)sum / (float)(trimHigh - trimLow);

    float voltage = 0.0f;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (s_adcCaliEnabled && s_adcCaliHandle) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_adcCaliHandle, (int)(avgRaw + 0.5f), &mv) == ESP_OK) {
            voltage = mv / 1000.0f;
        } else {
            voltage = avgRaw / 4095.0f * 3.3f;
        }
    } else
#endif
    {
        voltage = avgRaw / 4095.0f * 3.3f;
    }

    float rawBatteryV = voltage * BATTERY_DIVIDER_FACTOR;

    gApp.batteryVoltage = rawBatteryV;
    gApp.batteryPercent = voltageToPercent(rawBatteryV);

    // 저전압 자동 셧다운: 3.5V 연속 3회
    if (rawBatteryV <= BATTERY_SHUTDOWN_VOLTAGE) {
        s_lowVoltageCount++;
        if (s_lowVoltageCount >= BATTERY_SHUTDOWN_COUNT) {
            ESP_LOGW(TAG, "Battery critical: %.3fV (%d consecutive) - shutting down",
                     rawBatteryV, s_lowVoltageCount);
            systemPowerOff();
        }
    } else {
        s_lowVoltageCount = 0;
    }

    ESP_LOGI(TAG, "BAT %.3fV %.1f%%", rawBatteryV, gApp.batteryPercent);
}

// ============================================================
// IMU TASK (100Hz)
// ============================================================

static void imuTask(void *arg)
{
    // IMU 초기화 대기
    while (!imuIsReady()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 센서 안정화 대기 후 자동 캘리브레이션
    vTaskDelay(pdMS_TO_TICKS(500));
    if (imuCalibrate(100)) {
        gApp.imuCalibration = imuGetCalibration();
    }

    int tempCounter = 0;

    for (;;) {
        ImuData data;
        if (imuRead(&data)) {
            gApp.imuData = data;
            gApp.imuReady = true;
        }

        // 1초마다 온도 읽기
        if (++tempCounter >= 100) {
            tempCounter = 0;
            float temp;
            if (imuReadTemperature(&temp)) {
                gApp.imuData.temperature = temp;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
    }
}

static void startImuTask(void)
{
    xTaskCreatePinnedToCore(imuTask, "imu", 4096, NULL, 2, NULL, 1);
}

// ============================================================
// HELPER FUNCTIONS FOR MODULES
// ============================================================

/**
 * @brief Get current simulation lap index (for serial_commands)
 */
int getSimCurrentLapIdx(void) {
    return getSimulationState().currentLapIdx;
}

/**
 * @brief Get current simulation point index (for serial_commands)
 */
int getSimCurrentPointIdx(void) {
    return getSimulationState().currentPointIdx;
}


/**
 * @brief Update display data from current state
 */
void updateDisplayData(const GPSPoint& point, const DeltaResult& delta,
                       unsigned long lapTimeMs, float refTimeSec) {
    if (gApp.isSimulationMode()) {
        gframe.sats = SIM_SATELLITE_COUNT;
        gframe.fix = 1;
    } else {
        UBloxData ubx = getUBloxData();
        gframe.sats = ubx.satellites;
        gframe.fix = (ubx.fixType >= 2) ? 1 : 0;
    }
    gframe.bearing = (uint16_t)point.headingDeg;
    gframe.lon = (int32_t)(point.lng * 1e7);
    gframe.lat = (int32_t)(point.lat * 1e7);
    gframe.speed = (uint32_t)(point.speedKmh * 1000);

    lframe.time = lapTimeMs;
    lframe.bestLapMs = gApp.bestLapTimeMs;
    lframe.hasBestLap = gApp.hasValidReferenceLap && (gApp.bestLapTimeMs < UINT32_MAX);
    lframe.refTimeMs = (uint32_t)(refTimeSec * 1000);

    // Suppress delta if no valid reference
    if (!gApp.hasValidReferenceLap) {
        lframe.delta = 0;
    } else {
        lframe.delta = (int32_t)(delta.deltaSeconds * 1000);
    }

    if (std::isfinite(delta.distanceMeters) && delta.distanceMeters < (MAX_PROJECTION_DISTANCE_M * 2.0f)) {
        lframe.dist_start = (uint16_t)delta.distanceMeters;
    } else {
        lframe.dist_start = 0;
    }
    lframe.pts = gApp.referenceLap.points.size();

    // GPS 시간으로 시스템 시계 + RTC 동기화 (GPS 모드, 최초 1회)
    // gpsTimeSet은 RTC에서 이미 설정될 수 있으므로 별도 플래그 사용
    static bool s_gpsClockSynced = false;
    if (!gApp.isSimulationMode() && !s_gpsClockSynced) {
        UBloxData ubx = getUBloxData();
        if (ubx.timeValid && ubx.year >= 2024) {
            struct tm gpsTime = {};
            gpsTime.tm_year = ubx.year - 1900;
            gpsTime.tm_mon  = ubx.month - 1;
            gpsTime.tm_mday = ubx.day;
            gpsTime.tm_hour = ubx.hour;
            gpsTime.tm_min  = ubx.minute;
            gpsTime.tm_sec  = ubx.second;
            time_t utcEpoch = mktime(&gpsTime);
            struct timeval tv = { .tv_sec = utcEpoch, .tv_usec = 0 };
            settimeofday(&tv, NULL);
            gApp.gpsTimeSet = true;
            s_gpsClockSynced = true;
            ESP_LOGI(TAG, "System clock set from GPS: %04d-%02d-%02d %02d:%02d:%02d UTC",
                     ubx.year, ubx.month, ubx.day, ubx.hour, ubx.minute, ubx.second);

            // GPS 시간을 RTC에 저장 (다음 부팅 시 즉시 사용)
            if (rtcIsReady()) {
                rtcWrite(&gpsTime);
            }
        }
    }

    // 시스템 시계에서 KST(UTC+9) 시간 표시 — 1초에 1번만 업데이트
    // Time display source policy:
    // 1) Prefer RTC whenever RTC holds valid time.
    // 2) If RTC is invalid/reset, fallback to valid system clock.
    // 3) Otherwise keep time hidden.
    static unsigned long s_lastRtcRetryMs = 0;
    static unsigned long lastTimeUpdateMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastTimeUpdateMs >= 1000) {
        lastTimeUpdateMs = nowMs;

        struct tm utcSource = {};
        bool hasRtcTime = readValidRtcUtc(&utcSource);

        if (!hasRtcTime && !gApp.gpsTimeSet &&
            (nowMs - s_lastRtcRetryMs >= RTC_RETRY_INTERVAL_MS)) {
            s_lastRtcRetryMs = nowMs;
            (void)trySyncSystemClockFromRTC();
            hasRtcTime = readValidRtcUtc(&utcSource);
        }

        if (hasRtcTime) {
            gApp.gpsTimeSet = true;
            time_t utc = mktime(&utcSource);
            utc += 9 * 3600;  // UTC -> KST
            struct tm kst = {};
            gmtime_r(&utc, &kst);
            tframe.hours   = kst.tm_hour;
            tframe.minutes = kst.tm_min;
            tframe.seconds = kst.tm_sec;
            tframe.year    = kst.tm_year + 1900;
            tframe.month   = kst.tm_mon + 1;
            tframe.date    = kst.tm_mday;
        } else {
            time_t nowUtc = 0;
            time(&nowUtc);
            if (isSystemClockValidUtc(nowUtc)) {
                nowUtc += 9 * 3600;  // UTC -> KST
                struct tm kst = {};
                gmtime_r(&nowUtc, &kst);
                tframe.hours   = kst.tm_hour;
                tframe.minutes = kst.tm_min;
                tframe.seconds = kst.tm_sec;
                tframe.year    = kst.tm_year + 1900;
                tframe.month   = kst.tm_mon + 1;
                tframe.date    = kst.tm_mday;
            } else {
                tframe.hours   = 0xFF;
                tframe.minutes = 0;
                tframe.seconds = 0;
                tframe.year    = 0;
                tframe.month   = 0;
                tframe.date    = 0;
            }
        }
    }

    tframe.lap = gApp.currentLapNumber;
    tframe.session = gApp.currentSessionNumber;

    gvalid = true;
    lvalid = true;
    tvalid = true;
}

/**
 * @brief Update top 3 laps for display
 */
void updateTop3Laps(uint16_t justCompletedLapNumber, uint32_t justCompletedLapTimeMs) {
    ESP_LOGI(TAG, "updateTop3Laps: lap=%u, time=%lu ms", justCompletedLapNumber, justCompletedLapTimeMs);
    ESP_LOGI(TAG, "Before: top3=[%u:%lu, %u:%lu, %u:%lu]",
             gApp.top3Laps[0].lapNumber, gApp.top3Laps[0].lapTimeMs,
             gApp.top3Laps[1].lapNumber, gApp.top3Laps[1].lapTimeMs,
             gApp.top3Laps[2].lapNumber, gApp.top3Laps[2].lapTimeMs);
    
    // Insert the just-completed lap into top 3 if it's faster
    for (int i = 0; i < 3; i++) {
        if (justCompletedLapTimeMs < gApp.top3Laps[i].lapTimeMs) {
            // Shift down to make room
            for (int j = 2; j > i; j--) {
                gApp.top3Laps[j] = gApp.top3Laps[j-1];
            }
            // Insert new lap
            gApp.top3Laps[i].lapNumber = justCompletedLapNumber;
            gApp.top3Laps[i].lapTimeMs = justCompletedLapTimeMs;
            ESP_LOGI(TAG, "Inserted at rank %d", i+1);
            break;
        }
    }
    
    ESP_LOGI(TAG, "After: top3=[%u:%lu, %u:%lu, %u:%lu]",
             gApp.top3Laps[0].lapNumber, gApp.top3Laps[0].lapTimeMs,
             gApp.top3Laps[1].lapNumber, gApp.top3Laps[1].lapTimeMs,
             gApp.top3Laps[2].lapNumber, gApp.top3Laps[2].lapTimeMs);
}

// Forward declaration
static bool convertStorableToLapData(const StorableLap& stored, LapData& out);

/**
 * @brief Handle lap completion
 */
void onLapComplete(unsigned long lapTimeMs) {
    uint16_t completedLapNumber = gApp.currentLapNumber;
    ESP_LOGI(TAG, "=== LAP %u COMPLETE: %.2fs ===", completedLapNumber, lapTimeMs / 1000.0f);

    // Finish recording and save
    StorableLap completedLap;
    if (finishRecordingLap(completedLap)) {
        saveLap(completedLap);

        // Check if new best
        if (lapTimeMs < gApp.bestLapTimeMs) {
            gApp.bestLapTimeMs = lapTimeMs;
            saveBestLap(completedLap);

            // 새 베스트랩을 referenceLap에 즉시 반영 (Phase 7)
            LapData newRef;
            if (convertStorableToLapData(completedLap, newRef)) {
                setReferenceLap(newRef);
                // 섹터 경계 거리 재계산
                if (!gApp.isSimulationMode()) {
                    updateSectorDistancesFromReference(
                        gApp.referenceLap.points.data(),
                        gApp.referenceLap.cumulativeDistances.data(),
                        (int)gApp.referenceLap.points.size(),
                        getGPSSectorBoundaries(),
                        getGPSSectorBoundaryCount());
                }
                ESP_LOGI(TAG, "NEW BEST LAP! Reference updated (%d pts)",
                         (int)gApp.referenceLap.points.size());
            } else {
                ESP_LOGW(TAG, "NEW BEST LAP! But reference conversion failed");
            }
        }
    }

    // Update top 3 laps (only on lap completion - not every frame!)
    updateTop3Laps(completedLapNumber, lapTimeMs);

    // Set completed lap display
    lframe.lastCompletedLapMs = lapTimeMs;
    lframe.lapCompleteDisplayEndMs = millis() + LAP_COMPLETE_DISPLAY_MS;

    // Increment lap number
    gApp.currentLapNumber++;

    // Mark that we now have a valid reference
    if (!gApp.hasValidReferenceLap) {
        gApp.hasValidReferenceLap = true;
    }

    // Reset for next lap
    gApp.lastValidSegmentIndex = -1;
    resetDeltaHistory();
    resetCrossingState();
    resetSectorTiming();

    // Start new recording
    startRecordingLap(gApp.currentSessionNumber, gApp.currentLapNumber);
}

/**
 * @brief Load reference lap from storage (best lap)
 */
// StorableLap → LapData 변환
static bool convertStorableToLapData(const StorableLap& stored, LapData& out) {
    out.clear();
    out.points.reserve(stored.points.size());

    float maxSpeed = 0.0f;
    float totalSpeed = 0.0f;

    for (const auto& sp : stored.points) {
        GPSPoint point;
        point.lat = sp.lat / 1e7;
        point.lng = sp.lng / 1e7;
        point.lapTimeMs = sp.lapTimeMs;
        point.speedKmh = sp.speedX10 / 10.0f;
        point.headingDeg = sp.heading * (360.0f / 256.0f);
        point.gpsTimeMs = sp.lapTimeMs;

        out.points.push_back(point);

        totalSpeed += point.speedKmh;
        if (point.speedKmh > maxSpeed) maxSpeed = point.speedKmh;
    }

    if (out.points.empty()) {
        return false;
    }

    out.totalTimeMs = stored.totalTimeMs;
    out.maxSpeedKmh = maxSpeed;
    out.avgSpeedKmh = totalSpeed / out.points.size();

    calculateCumulativeDistances(out);
    return true;
}

bool loadReferenceLapFromStorage(void) {
    StorableLap stored;
    if (!loadBestLap(stored)) {
        return false;
    }

    if (!convertStorableToLapData(stored, gApp.referenceLap)) {
        return false;
    }

    gApp.bestLapTimeMs = gApp.referenceLap.totalTimeMs;
    return true;
}

// ============================================================
// INITIALIZATION
// ============================================================

static void init_storage(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted");
    }

    // Initialize SD card (FAT32, SDMMC 1-bit)
    if (sdcardInit()) {
        gApp.sdCardMounted = true;
        ESP_LOGI(TAG, "SD card: mounted");
    } else {
        gApp.sdCardMounted = false;
        ESP_LOGW(TAG, "SD card: not available");
    }
}

static void app_init(void) {
    LOG_INTRAM("boot");

    // 전원 래치 즉시 실행 (배터리 모드: 버튼 놓기 전에 SYS_EN 래치 필요)
    initPowerLatch();

    // OTA 롤백 확인 (즉시, <1ms)
    ota::confirmBoot();

    // 디스플레이 즉시 초기화 (깨진 화면 최소화)
    initDisplay();
    setupUI();
    createStartupScreen();
    ESP_LOGI(TAG, "Display: OK");
    LOG_INTRAM("display");

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  GPS Lap Timer %s (ESP-IDF)", APP_VERSION);
    ESP_LOGI(TAG, "  Waveshare ESP32-S3-Touch-LCD-3.49");
    ESP_LOGI(TAG, "========================================");

    // 전원 관리 설정: DFS 활성화 (160MHz ↔ 80MHz)
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 80,
        .light_sleep_enable = false
    };
    esp_err_t pm_ret = esp_pm_configure(&pm_config);
    if (pm_ret == ESP_OK) {
        ESP_LOGI(TAG, "Power Management: DFS enabled (80-160MHz)");
    } else {
        ESP_LOGW(TAG, "Power Management: configure failed (%s)", esp_err_to_name(pm_ret));
    }
#endif

    // Initialize storage (NVS + SPIFFS + SD card)
    init_storage();
    LOG_INTRAM("storage");

    // Initialize lap storage
    if (!initLapStorage()) {
        ESP_LOGE(TAG, "Storage: FAILED");
    } else {
        ESP_LOGI(TAG, "Storage: OK");
    }
    LOG_INTRAM("lap_storage");

    // SPIFFS에서 설정 로드 (phoneNumber 등)
    loadSettings();

    // SD 카드 읽기는 WiFi 시작 전에 수행 (WiFi가 내부 RAM을 많이 차지하므로)
    // Load reference lap
    ESP_LOGI(TAG, "Loading reference lap...");
    if (loadReferenceLapFromStorage()) {
        ESP_LOGI(TAG, "  From storage: %d pts, %.2fs (BEST)",
                 (int)gApp.referenceLap.points.size(), gApp.referenceLap.totalTimeMs / 1000.0f);
        gApp.hasValidReferenceLap = true;
    } else {
        ESP_LOGI(TAG, "  No saved reference - starting fresh");
        gApp.hasValidReferenceLap = false;
        gApp.bestLapTimeMs = UINT32_MAX;
    }

    // Get session number
    gApp.currentSessionNumber = getNextSessionId();
    ESP_LOGI(TAG, "Session: %u", gApp.currentSessionNumber);
    LOG_INTRAM("pre-wifi");

    // WiFi 초기화 (SETTINGS 페이지에서만 시작)
    initWifiPortal();
    LOG_INTRAM("wifi-init");

    // Initialize finish line detection
    initFinishLine();
    ESP_LOGI(TAG, "Finish line: %s", isFinishLineConfigured() ? "CONFIGURED" : "NOT SET");

    // Initialize RTC (PCF85063 on sensor I2C bus)
    i2c_master_bus_handle_t sensorBus = getSensorI2CBus();
    if (sensorBus && rtcInit(sensorBus)) {
        struct tm rtcTime = {};
        if (rtcRead(&rtcTime) && rtcTime.tm_year >= (2024 - 1900)) {
            time_t utcEpoch = mktime(&rtcTime);
            struct timeval tv = { .tv_sec = utcEpoch, .tv_usec = 0 };
            settimeofday(&tv, NULL);
            gApp.gpsTimeSet = true;
            ESP_LOGI(TAG, "System clock set from RTC: %04d-%02d-%02d %02d:%02d:%02d UTC",
                     rtcTime.tm_year + 1900, rtcTime.tm_mon + 1, rtcTime.tm_mday,
                     rtcTime.tm_hour, rtcTime.tm_min, rtcTime.tm_sec);
        } else {
            ESP_LOGI(TAG, "RTC: no valid time stored");
        }
    } else {
        ESP_LOGW(TAG, "RTC: init failed or I2C bus unavailable");
    }

    // Initialize IMU (QMI8658C on sensor I2C bus)
    if (sensorBus && imuInit(sensorBus)) {
        ESP_LOGI(TAG, "IMU: QMI8658C OK");
    } else {
        ESP_LOGW(TAG, "IMU: init failed or I2C bus unavailable");
    }

    // Initialize serial command handler
    initSerialCommands();

    // GPS 모듈 전원 핀 초기화 후 즉시 활성화 (부팅 즉시 위성 수신 시작)
    gpio_config_t gps_en_gpio = {};
    gps_en_gpio.intr_type = GPIO_INTR_DISABLE;
    gps_en_gpio.mode = GPIO_MODE_OUTPUT;
    gps_en_gpio.pin_bit_mask = (1ULL << GPS_ENABLE_PIN);
    gps_en_gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gps_en_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&gps_en_gpio);
    enableGPSModule();
    ESP_LOGI(TAG, "GPS module: ON (boot-time init)");

    // 모드 초기화는 시작 화면에서 모드 선택 후 수행
    resetDeltaHistory();

    // Initialize battery ADC (GPIO4) + 최초 1회 측정
    initBatteryADC();
    readBattery();

    // IMU 태스크 시작 (100Hz 읽기 + 자동 캘리브레이션)
    startImuTask();

    // PWR 버튼 GPIO 초기화 (GPIO16, active low, 풀업)
    gpio_config_t pwr_gpio = {};
    pwr_gpio.intr_type = GPIO_INTR_DISABLE;
    pwr_gpio.mode = GPIO_MODE_INPUT;
    pwr_gpio.pin_bit_mask = (1ULL << PWR_BUTTON_PIN);
    pwr_gpio.pull_up_en = GPIO_PULLUP_ENABLE;
    pwr_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&pwr_gpio);
    ESP_LOGI(TAG, "PWR button: GPIO%d", PWR_BUTTON_PIN);

    ESP_LOGI(TAG, "Type 'h' for command help");
}

// ============================================================
// PWR BUTTON HANDLING
// ============================================================

static unsigned long s_pwrButtonPressStart = 0;
static bool s_pwrButtonWasPressed = false;

static void checkPowerButton(void)
{
    bool pressed = (gpio_get_level((gpio_num_t)PWR_BUTTON_PIN) == 0);
    unsigned long now = (unsigned long)(esp_timer_get_time() / 1000);

    if (pressed && !s_pwrButtonWasPressed) {
        // 버튼 눌림 시작
        s_pwrButtonPressStart = now;
        s_pwrButtonWasPressed = true;
    } else if (pressed && s_pwrButtonWasPressed) {
        // 길게 누르고 있는 중
        if ((now - s_pwrButtonPressStart) >= POWER_OFF_HOLD_MS) {
            ESP_LOGI(TAG, "PWR button long press - powering off");
            systemPowerOff();
            s_pwrButtonWasPressed = false;
        }
    } else if (!pressed) {
        s_pwrButtonWasPressed = false;
    }
}

// ============================================================
// MAIN TASK
// ============================================================

static void main_task(void *pvParameters) {
    app_init();

    unsigned long lastDisplayUpdate = 0;
    unsigned long lastBatteryRead = 0;
    bool modeInitialized = false;

    while (1) {
        // 시작 화면 중에는 GPS/시뮬레이션 처리 건너뜀
        // (GPS 폴링은 updateStartupScreen에서 직접 수행)
        if (!isStartupScreenActive()) {
            if (isGpsStatusOnlyMode()) {
                // GPS STATUS 진단 모드: UART 폴링만 수행 (랩타이머 비활성)
                updateUBloxGPS();
            } else {
                // 시작 화면에서 모드 선택 후 최초 1회 초기화
                if (!modeInitialized) {
                    ESP_LOGI(TAG, "Mode: %s", gApp.isSimulationMode() ? "EMULATION" : "LAPTIMER");
                    if (gApp.isSimulationMode()) {
                        initializeSimulation();
                    } else {
                        initializeGPSMode();
                    }
                    modeInitialized = true;
                }

                if (gApp.currentGpsMode == GPSMode::SIMULATION) {
                    processSimulation();
                } else {
                    processRealGPS();
                }
            }
        }

        // PWR 버튼 체크 (배터리 모드 전원 오프)
        checkPowerButton();

        // Handle serial commands
        handleSerialCommands();

        // Update display at fixed rate
        unsigned long now = millis();
        if (now - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL_MS) {
            displayLoop();
            lastDisplayUpdate = now;
        }

        // 배터리 측정 (60초마다, ~2.6ms 소요)
        if (now - lastBatteryRead >= BATTERY_READ_INTERVAL_MS) {
            readBattery();
            lastBatteryRead = now;
        }

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ============================================================
// APP MAIN (ESP-IDF Entry Point)
// ============================================================

extern "C" void app_main(void) {
    // Create main task with larger stack for C++ STL
    xTaskCreatePinnedToCore(
        main_task,
        "main_task",
        MAIN_TASK_STACK_SIZE,
        NULL,
        MAIN_TASK_PRIORITY,
        NULL,
        MAIN_TASK_CORE
    );
}
