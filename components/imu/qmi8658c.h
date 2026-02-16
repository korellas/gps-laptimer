/**
 * @file qmi8658c.h
 * @brief QMI8658C 6-axis IMU driver (ESP-IDF I2C master API)
 *
 * I2C address: 0x6B
 * Sensor I2C bus: GPIO47(SDA) / GPIO48(SCL), shared with TCA9554/PCF85063
 * Accel: ±8g, 125Hz | Gyro: ±512°/s, 125Hz
 *
 * ImuData/ImuCalibration 구조체는 types.h에 정의 (순환 의존 방지)
 */

#pragma once

// Forward declaration (avoid pulling driver/i2c_master.h into all consumers)
typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

// Forward declare structs (defined in types.h)
struct ImuData;
struct ImuCalibration;

// ============================================================
// Public API
// ============================================================

#ifdef __cplusplus
extern "C" {
#endif

// IMU 초기화 (WHO_AM_I 확인, 센서 설정)
bool imuInit(i2c_master_bus_handle_t bus);

// 6축 데이터 읽기 (burst read)
bool imuRead(ImuData* data);

// 칩 온도 읽기
bool imuReadTemperature(float* tempC);

// 초기화 완료 여부
bool imuIsReady(void);

// 정지 상태 캘리브레이션 (numSamples 평균, 중력 보정)
bool imuCalibrate(int numSamples);

// 현재 캘리브레이션 데이터 반환
ImuCalibration imuGetCalibration(void);

#ifdef __cplusplus
}
#endif
