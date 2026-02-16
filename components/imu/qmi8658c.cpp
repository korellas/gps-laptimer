/**
 * @file qmi8658c.cpp
 * @brief QMI8658C 6-axis IMU driver (ESP-IDF I2C master API)
 *
 * Register map (rev 0.9):
 *   0x00  WHO_AM_I     (expected 0x05)
 *   0x01  REVISION
 *   0x02  CTRL1        addr auto-increment, SPI/I2C config
 *   0x03  CTRL2        accel FS + ODR
 *   0x04  CTRL3        gyro FS + ODR
 *   0x06  CTRL5        LPF config
 *   0x08  CTRL7        sensor enable
 *   0x2E  STATUS0      data ready flags
 *   0x33  TEMP_L       temperature low byte
 *   0x34  TEMP_H       temperature high byte
 *   0x35-0x40           accel XYZ + gyro XYZ (12 bytes, burst)
 *   0x60  RESET        soft reset (write 0xB0)
 */

#include "qmi8658c.h"
#include "types.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>

static const char* TAG = "IMU";

// ============================================================
// QMI8658C 레지스터 정의
// ============================================================

static constexpr uint8_t QMI8658C_ADDR         = 0x6B;
static constexpr uint8_t QMI8658C_WHO_AM_I_VAL = 0x05;

static constexpr uint8_t REG_WHO_AM_I   = 0x00;
static constexpr uint8_t REG_REVISION   = 0x01;
static constexpr uint8_t REG_CTRL1      = 0x02;
static constexpr uint8_t REG_CTRL2      = 0x03;  // Accel FS + ODR
static constexpr uint8_t REG_CTRL3      = 0x04;  // Gyro FS + ODR
static constexpr uint8_t REG_CTRL5      = 0x06;  // LPF
static constexpr uint8_t REG_CTRL7      = 0x08;  // Sensor enable
static constexpr uint8_t REG_STATUS0    = 0x2E;  // Data ready
static constexpr uint8_t REG_TEMP_L     = 0x33;
static constexpr uint8_t REG_TEMP_H     = 0x34;
static constexpr uint8_t REG_DATA_START = 0x35;  // AX_L
static constexpr uint8_t REG_RESET      = 0x60;

static constexpr uint8_t RESET_VALUE    = 0xB0;  // Soft reset command

// CTRL1: address auto-increment 활성화
static constexpr uint8_t CTRL1_VALUE = 0x40;

// CTRL2: Accel ±8g (FS=0b10), 125Hz ODR (0x06)
// FS: 00=±2g, 01=±4g, 10=±8g, 11=±16g
// ODR: 0x06=125Hz (100Hz에 가장 가까운 설정)
static constexpr uint8_t CTRL2_VALUE = (0x02 << 4) | 0x06;

// CTRL3: Gyro ±512°/s (FS=0b101), 125Hz ODR (0x06)
// FS: 000=±16, 001=±32, 010=±64, 011=±128, 100=±256, 101=±512, 110=±1024, 111=±2048
static constexpr uint8_t CTRL3_VALUE = (0x05 << 4) | 0x06;

// CTRL5: Accel + Gyro LPF 활성화
static constexpr uint8_t CTRL5_VALUE = 0x11;

// CTRL7: Accel enable (bit0) + Gyro enable (bit1)
static constexpr uint8_t CTRL7_VALUE = 0x03;

// 변환 계수
static constexpr float ACCEL_SENSITIVITY = 4096.0f;   // LSB/g (±8g)
static constexpr float GYRO_SENSITIVITY  = 64.0f;     // LSB/°/s (±512°/s)
static constexpr float TEMP_SENSITIVITY  = 256.0f;    // LSB/°C

static constexpr int DATA_LENGTH = 12;  // 6 accel + 6 gyro bytes

// ============================================================
// Module State
// ============================================================

static i2c_master_dev_handle_t s_imu_dev = nullptr;
static ImuCalibration s_calibration = {};
static bool s_calibrationEnabled = true;

// ============================================================
// I2C 레지스터 헬퍼
// ============================================================

static esp_err_t imuReadRegs(uint8_t regAddr, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(s_imu_dev, &regAddr, 1, data, len, 100);
}

static esp_err_t imuWriteReg(uint8_t regAddr, uint8_t value)
{
    uint8_t buf[2] = { regAddr, value };
    return i2c_master_transmit(s_imu_dev, buf, 2, 100);
}

// ============================================================
// Public API
// ============================================================

bool imuInit(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return false;
    }

    // I2C 디바이스 등록
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = QMI8658C_ADDR;
    dev_cfg.scl_speed_hz = 400000;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_imu_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(err));
        s_imu_dev = nullptr;
        return false;
    }

    // WHO_AM_I 확인 (재시도 3회)
    uint8_t whoami = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        err = imuReadRegs(REG_WHO_AM_I, &whoami, 1);
        if (err == ESP_OK && whoami == QMI8658C_WHO_AM_I_VAL) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (whoami != QMI8658C_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X (expected 0x%02X)", whoami, QMI8658C_WHO_AM_I_VAL);
        s_imu_dev = nullptr;
        return false;
    }

    // Soft reset
    imuWriteReg(REG_RESET, RESET_VALUE);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Reset 후 WHO_AM_I 재확인
    err = imuReadRegs(REG_WHO_AM_I, &whoami, 1);
    if (err != ESP_OK || whoami != QMI8658C_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I after reset failed: 0x%02X", whoami);
        s_imu_dev = nullptr;
        return false;
    }

    // 센서 설정
    imuWriteReg(REG_CTRL1, CTRL1_VALUE);   // Auto-increment
    imuWriteReg(REG_CTRL2, CTRL2_VALUE);   // Accel: ±8g, 125Hz
    imuWriteReg(REG_CTRL3, CTRL3_VALUE);   // Gyro: ±512°/s, 125Hz
    imuWriteReg(REG_CTRL5, CTRL5_VALUE);   // LPF enable
    imuWriteReg(REG_CTRL7, CTRL7_VALUE);   // Accel + Gyro enable

    // 센서 안정화 대기
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t revision = 0;
    imuReadRegs(REG_REVISION, &revision, 1);

    ESP_LOGI(TAG, "QMI8658C init OK (WHO_AM_I=0x%02X, REV=0x%02X)", whoami, revision);
    ESP_LOGI(TAG, "  Accel: +/-8g, 125Hz | Gyro: +/-512dps, 125Hz");

    return true;
}

bool imuRead(ImuData* data)
{
    if (!s_imu_dev || !data) return false;

    // Burst read: AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H, GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H
    uint8_t raw[DATA_LENGTH];
    esp_err_t err = imuReadRegs(REG_DATA_START, raw, DATA_LENGTH);
    if (err != ESP_OK) {
        data->valid = false;
        return false;
    }

    // 16-bit signed (little-endian)
    int16_t rawAx = (int16_t)(raw[0]  | (raw[1]  << 8));
    int16_t rawAy = (int16_t)(raw[2]  | (raw[3]  << 8));
    int16_t rawAz = (int16_t)(raw[4]  | (raw[5]  << 8));
    int16_t rawGx = (int16_t)(raw[6]  | (raw[7]  << 8));
    int16_t rawGy = (int16_t)(raw[8]  | (raw[9]  << 8));
    int16_t rawGz = (int16_t)(raw[10] | (raw[11] << 8));

    // 물리 단위 변환
    data->rawAccelX = rawAx / ACCEL_SENSITIVITY;
    data->rawAccelY = rawAy / ACCEL_SENSITIVITY;
    data->rawAccelZ = rawAz / ACCEL_SENSITIVITY;

    data->gyroX = rawGx / GYRO_SENSITIVITY;
    data->gyroY = rawGy / GYRO_SENSITIVITY;
    data->gyroZ = rawGz / GYRO_SENSITIVITY;

    // 캘리브레이션 적용
    if (s_calibrationEnabled && s_calibration.calibrated) {
        data->accelX = data->rawAccelX - s_calibration.accelOffsetX;
        data->accelY = data->rawAccelY - s_calibration.accelOffsetY;
        data->accelZ = data->rawAccelZ - s_calibration.accelOffsetZ;
        data->gyroX -= s_calibration.gyroOffsetX;
        data->gyroY -= s_calibration.gyroOffsetY;
        data->gyroZ -= s_calibration.gyroOffsetZ;
    } else {
        data->accelX = data->rawAccelX;
        data->accelY = data->rawAccelY;
        data->accelZ = data->rawAccelZ;
    }

    data->valid = true;
    data->timestampMs = (uint32_t)(esp_timer_get_time() / 1000);

    return true;
}

bool imuReadTemperature(float* tempC)
{
    if (!s_imu_dev || !tempC) return false;

    uint8_t raw[2];
    esp_err_t err = imuReadRegs(REG_TEMP_L, raw, 2);
    if (err != ESP_OK) return false;

    int16_t rawTemp = (int16_t)(raw[0] | (raw[1] << 8));
    *tempC = rawTemp / TEMP_SENSITIVITY;
    return true;
}

bool imuIsReady(void)
{
    return s_imu_dev != nullptr;
}

bool imuCalibrate(int numSamples)
{
    if (!s_imu_dev || numSamples < 10) return false;

    ESP_LOGI(TAG, "Calibrating (%d samples)...", numSamples);

    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    int validCount = 0;

    // 캘리브레이션 중 오프셋 적용 비활성화
    bool prevEnabled = s_calibrationEnabled;
    s_calibrationEnabled = false;

    for (int i = 0; i < numSamples; i++) {
        ImuData sample;
        if (imuRead(&sample)) {
            sumAx += sample.rawAccelX;
            sumAy += sample.rawAccelY;
            sumAz += sample.rawAccelZ;
            sumGx += sample.gyroX;
            sumGy += sample.gyroY;
            sumGz += sample.gyroZ;
            validCount++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_calibrationEnabled = prevEnabled;

    if (validCount < numSamples / 2) {
        ESP_LOGE(TAG, "Calibration failed: only %d/%d valid", validCount, numSamples);
        return false;
    }

    float n = (float)validCount;
    s_calibration.accelOffsetX = sumAx / n;
    s_calibration.accelOffsetY = sumAy / n;
    // Z축: 중력(~1g) 보정 — 정지 상태에서 Z ≈ 1.0g
    // 가장 큰 절대값을 가진 축을 중력축으로 판별
    float avgAx = sumAx / n;
    float avgAy = sumAy / n;
    float avgAz = sumAz / n;
    float absX = fabsf(avgAx), absY = fabsf(avgAy), absZ = fabsf(avgAz);

    if (absZ >= absX && absZ >= absY) {
        // Z축이 중력축
        s_calibration.accelOffsetZ = avgAz - (avgAz > 0 ? 1.0f : -1.0f);
    } else if (absY >= absX && absY >= absZ) {
        // Y축이 중력축
        s_calibration.accelOffsetY = avgAy - (avgAy > 0 ? 1.0f : -1.0f);
        s_calibration.accelOffsetZ = avgAz;
    } else {
        // X축이 중력축
        s_calibration.accelOffsetX = avgAx - (avgAx > 0 ? 1.0f : -1.0f);
        s_calibration.accelOffsetZ = avgAz;
    }

    s_calibration.gyroOffsetX = sumGx / n;
    s_calibration.gyroOffsetY = sumGy / n;
    s_calibration.gyroOffsetZ = sumGz / n;

    s_calibration.calibrated = true;
    s_calibration.samplesUsed = validCount;

    ESP_LOGI(TAG, "Calibration OK (%d samples)", validCount);
    ESP_LOGI(TAG, "  Accel offset: X=%+.4f Y=%+.4f Z=%+.4f g",
             s_calibration.accelOffsetX, s_calibration.accelOffsetY,
             s_calibration.accelOffsetZ);
    ESP_LOGI(TAG, "  Gyro offset:  X=%+.2f Y=%+.2f Z=%+.2f dps",
             s_calibration.gyroOffsetX, s_calibration.gyroOffsetY,
             s_calibration.gyroOffsetZ);

    return true;
}

ImuCalibration imuGetCalibration(void)
{
    return s_calibration;
}
