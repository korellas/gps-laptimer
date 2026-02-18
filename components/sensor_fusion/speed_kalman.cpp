/**
 * @file speed_kalman.cpp
 * @brief 1D scalar Kalman filter for GPS+IMU speed fusion
 *
 * State: vehicle speed (m/s)
 * Predict: integrate forward acceleration from calibrated IMU
 * Update: correct with GPS ground speed measurement
 */

#include "sensor_fusion.h"
#include <cmath>

// ============================================================
// TUNING PARAMETERS
// ============================================================

// Process noise: how much we distrust the accelerometer per second
// Higher = trust GPS more, lower = trust accelerometer more
static constexpr float KF_Q = 2.0f;      // m/s^2 variance per second

// Measurement noise: GPS speed accuracy (u-blox claims ±0.05 m/s)
static constexpr float KF_R = 0.01f;     // (m/s)^2

// Initial error covariance
static constexpr float KF_P_INIT = 1.0f;

// Speed clamp
static constexpr float KF_MAX_SPEED_MS = 100.0f;  // 360 km/h
static constexpr float KF_MIN_SPEED_MS = 0.0f;

// ============================================================
// STATE
// ============================================================

static float s_speed = 0.0f;    // Estimated speed (m/s)
static float s_P = KF_P_INIT;   // Error covariance
static bool  s_active = false;

// ============================================================
// API
// ============================================================

void speedKfInit(float speedMs)
{
    s_speed = speedMs;
    s_P = KF_P_INIT;
    s_active = true;
}

void speedKfReset(void)
{
    s_speed = 0.0f;
    s_P = KF_P_INIT;
    s_active = false;
}

void speedKfPredict(float fwdAccelMs2, float dt)
{
    if (!s_active || dt <= 0.0f) return;

    s_speed += fwdAccelMs2 * dt;
    s_P += KF_Q * dt;

    // Clamp to reasonable range
    if (s_speed < KF_MIN_SPEED_MS) s_speed = KF_MIN_SPEED_MS;
    if (s_speed > KF_MAX_SPEED_MS) s_speed = KF_MAX_SPEED_MS;
}

void speedKfUpdate(float gpsSpeedMs)
{
    if (!s_active) return;

    float K = s_P / (s_P + KF_R);
    s_speed += K * (gpsSpeedMs - s_speed);
    s_P *= (1.0f - K);

    // Clamp
    if (s_speed < KF_MIN_SPEED_MS) s_speed = KF_MIN_SPEED_MS;
    if (s_speed > KF_MAX_SPEED_MS) s_speed = KF_MAX_SPEED_MS;
}

float speedKfGetSpeedMs(void)
{
    return s_speed;
}

float speedKfGetSpeedKmh(void)
{
    return s_speed * 3.6f;
}

bool speedKfIsActive(void)
{
    return s_active;
}
