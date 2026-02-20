/**
 * @file imu_page.cpp
 * @brief IMU calibration visualization page.
 *
 * Diagnostic page showing:
 *   - G-force scatter plot (forward/lateral) with moving dot
 *   - Calibration status (R matrix validity, residual, gravity vector)
 *   - Z-axis vertical bar indicator
 *
 * Runs axis calibration feed+solve independently of gps_processor,
 * so calibration works even from MODE_SELECT → IMU STATUS directly.
 *
 * Access: MODE_SELECT → swipe to "< IMU STATUS >" → tap
 * Exit:   TAP or SWIPE_DOWN → MODE_SELECT
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "ublox_gps.h"
#include "sensor_fusion.h"
#include "types.h"

#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "ImuPage";

struct ImuPage : Page {
    unsigned long lastCalibAttemptMs = 0;
    static constexpr unsigned long CALIB_INTERVAL_MS = 10000;

    ImuPage()
        : Page(PageId::IMU_STATUS,
               /*gps=*/true, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/false, SleepPolicy::NO_SLEEP)
    {}

    void onEnter(Page* from) override {
        applyPageVisibilityForPage(PageId::IMU_STATUS);

        // Initialize calibrator + load saved calibration
        axisCalibInit();
        if (gApp.imuCalibration.calibrated) {
            axisCalibSetGravity(gApp.imuCalibration.gravityX,
                                gApp.imuCalibration.gravityY,
                                gApp.imuCalibration.gravityZ);
        }
        if (axisCalibLoad()) {
            ESP_LOGI(TAG, "Loaded saved calibration (residual=%.3f)", axisCalibGetResidual());
        } else {
            ESP_LOGI(TAG, "No saved calibration, will collect while driving");
        }
        lastCalibAttemptMs = 0;
    }

    void onTick() override {
        if (updateUBloxGPS()) {
            // Feed calibration sample on every GPS frame
            UBloxData ubx = getUBloxData();
            if (gApp.imuReady && ubx.valid) {
                float aSensor[3] = {
                    gApp.imuData.accelX,
                    gApp.imuData.accelY,
                    gApp.imuData.accelZ
                };
                axisCalibFeedGPS(ubx.velNorthMps, ubx.velEastMps, ubx.velDownMps,
                                 ubx.speedKmh, ubx.iTOW, aSensor);
            }

            // Periodic solve attempt
            unsigned long now = (unsigned long)(esp_timer_get_time() / 1000ULL);
            if ((now - lastCalibAttemptMs) >= CALIB_INTERVAL_MS) {
                lastCalibAttemptMs = now;
                if (axisCalibSolve()) {
                    axisCalibSave();
                    ESP_LOGI(TAG, "Calibration solved! residual=%.3f N=%d",
                             axisCalibGetResidual(), axisCalibGetSampleCount());
                }
            }
        }
    }

    void onUpdate() override {
        updateImuDisplay();
    }

    void onGesture(Gesture g) override {
        if (g == Gesture::TAP || g == Gesture::SWIPE_DOWN) {
            gPageManager.navigateTo(PageId::MODE_SELECT);
        }
    }
};

static ImuPage s_page;

Page* createImuPage() { return &s_page; }
