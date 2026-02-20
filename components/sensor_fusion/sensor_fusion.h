/**
 * @file sensor_fusion.h
 * @brief GPS+IMU sensor fusion: axis calibration (Wahba) + 1D speed Kalman filter
 *
 * Axis calibration: GPS NED velocity differentiation vs IMU acceleration
 *   → Wahba's problem (SVD) → rotation matrix R (sensor frame → NED)
 *
 * Speed Kalman filter: 1D scalar filter
 *   Predict: integrate forward acceleration from calibrated IMU (100Hz)
 *   Update:  GPS speed measurement (10Hz)
 *   GPS outage: predict-only → maintains speed estimate in tunnels
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// AXIS CALIBRATOR (Wahba's problem)
// ============================================================

/**
 * @brief Initialize the axis calibrator.
 * Resets sample buffer and internal state.
 */
void axisCalibInit(void);

/**
 * @brief Set the gravity vector in calibrated sensor frame (g units).
 * Must be called before feeding samples. Obtained from boot calibration.
 * e.g., [0, 0, 1.0] for Z-up mounting.
 */
void axisCalibSetGravity(float gx, float gy, float gz);

/**
 * @brief Reset sample buffer only (keeps saved calibration valid).
 * Call when re-entering PRE_TRACK after a session reset.
 */
void axisCalibReset(void);

/**
 * @brief Feed a GPS velocity frame for calibration sample collection.
 *
 * Internally differentiates consecutive GPS velocity vectors to get aNED,
 * pairs with the provided IMU acceleration (already zero-offset calibrated,
 * in g units), and stores the sample if quality criteria are met.
 *
 * @param velNorthMps  GPS North velocity (m/s)
 * @param velEastMps   GPS East velocity (m/s)
 * @param velDownMps   GPS Down velocity (m/s)
 * @param speedKmh     GPS ground speed (km/h) for quality gating
 * @param iTOW         GPS time of week (ms) for dt calculation
 * @param accelSensor  IMU accel [X,Y,Z] in g, zero-offset calibrated
 * @return true if a valid sample was added
 */
bool axisCalibFeedGPS(float velNorthMps, float velEastMps, float velDownMps,
                      float speedKmh, uint32_t iTOW,
                      const float accelSensor[3]);

/**
 * @brief Solve Wahba's problem with collected samples.
 * Computes rotation matrix R (sensor → NED) via SVD.
 * @return true if calibration succeeded (enough samples, low residual)
 */
bool axisCalibSolve(void);

/**
 * @brief Load calibration from SPIFFS (/spiffs/config/imu_fusion.json).
 * @return true if loaded and validated (orthogonal R, reasonable residual)
 */
bool axisCalibLoad(void);

/**
 * @brief Save current calibration to SPIFFS.
 * @return true if save succeeded
 */
bool axisCalibSave(void);

/**
 * @brief Check if a valid calibration exists (loaded or computed).
 */
bool axisCalibIsValid(void);

/**
 * @brief Get the rotation matrix R[3][3] (sensor → NED), row-major.
 * Returns pointer to internal static 9-float array. Valid only when axisCalibIsValid().
 */
const float* axisCalibGetR(void);

/**
 * @brief Get RMS residual of the calibration (m/s^2). Lower = better.
 */
float axisCalibGetResidual(void);

/**
 * @brief Get number of samples used in calibration.
 */
int axisCalibGetSampleCount(void);

// ============================================================
// 1D SPEED KALMAN FILTER
// ============================================================

/**
 * @brief Initialize (seed) the Kalman filter with a known speed.
 * @param speedMs  Initial speed estimate (m/s)
 */
void speedKfInit(float speedMs);

/**
 * @brief Reset the Kalman filter to inactive state.
 */
void speedKfReset(void);

/**
 * @brief Predict step: integrate forward acceleration.
 * Call at IMU rate (~100Hz).
 * @param fwdAccelMs2  Forward acceleration (m/s^2, positive = accelerating)
 * @param dt           Time step (seconds)
 */
void speedKfPredict(float fwdAccelMs2, float dt);

/**
 * @brief Update step: correct with GPS speed measurement.
 * Call at GPS rate (~10Hz) when GPS is valid.
 * @param gpsSpeedMs  GPS ground speed (m/s)
 */
void speedKfUpdate(float gpsSpeedMs);

/**
 * @brief Get current estimated speed in m/s.
 */
float speedKfGetSpeedMs(void);

/**
 * @brief Get current estimated speed in km/h.
 */
float speedKfGetSpeedKmh(void);

/**
 * @brief Check if the Kalman filter has been initialized and is active.
 */
bool speedKfIsActive(void);

#ifdef __cplusplus
}
#endif
