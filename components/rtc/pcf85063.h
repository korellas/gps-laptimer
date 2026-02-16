/**
 * @file pcf85063.h
 * @brief PCF85063 RTC driver for ESP-IDF I2C master
 *
 * I2C address: 0x51
 * Sensor I2C bus: GPIO47(SDA) / GPIO48(SCL), shared with TCA9554
 */

#pragma once

#include "driver/i2c_master.h"
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// RTC 초기화 (I2C 버스 핸들 전달)
bool rtcInit(i2c_master_bus_handle_t bus);

// RTC에서 시간 읽기 (UTC, struct tm으로 반환)
// OS bit가 1이면 false 반환 (시간 무효)
bool rtcRead(struct tm* time);

// RTC에 시간 쓰기 (UTC)
bool rtcWrite(const struct tm* time);

// RTC 초기화 완료 여부
bool rtcIsReady(void);

#ifdef __cplusplus
}
#endif
