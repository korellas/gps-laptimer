/**
 * @file pcf85063.cpp
 * @brief PCF85063 RTC driver (ESP-IDF I2C master API)
 *
 * Register map:
 *   0x00  Control_1
 *   0x01  Control_2
 *   0x04  Seconds  (BCD, bit7 = OS: clock integrity)
 *   0x05  Minutes  (BCD)
 *   0x06  Hours    (BCD, 24h format)
 *   0x07  Days     (BCD)
 *   0x08  Weekdays (0-6)
 *   0x09  Months   (BCD)
 *   0x0A  Years    (BCD, 0-99, offset from 2000)
 */

#include "pcf85063.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "RTC";

// PCF85063 I2C 주소 및 레지스터
static constexpr uint8_t PCF85063_ADDR     = 0x51;
static constexpr uint8_t REG_CONTROL_1     = 0x00;
static constexpr uint8_t REG_TIME_START    = 0x04;  // Seconds
static constexpr uint8_t REG_TIME_LENGTH   = 7;     // Seconds ~ Years

static i2c_master_dev_handle_t s_rtc_dev = nullptr;

// ============================================================
// BCD 변환 헬퍼
// ============================================================

static uint8_t bcdToDec(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static uint8_t decToBcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

// ============================================================
// I2C 레지스터 읽기/쓰기
// ============================================================

static esp_err_t rtcReadRegs(uint8_t regAddr, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(s_rtc_dev, &regAddr, 1, data, len, 100);
}

static esp_err_t rtcWriteRegs(uint8_t regAddr, const uint8_t* data, size_t len)
{
    uint8_t buf[8];
    buf[0] = regAddr;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(s_rtc_dev, buf, 1 + len, 100);
}

// ============================================================
// Public API
// ============================================================

bool rtcInit(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return false;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = PCF85063_ADDR;
    dev_cfg.scl_speed_hz = 400000;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_rtc_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(err));
        s_rtc_dev = nullptr;
        return false;
    }

    // Control_1 레지스터 읽기로 통신 확인
    uint8_t ctrl1 = 0;
    err = rtcReadRegs(REG_CONTROL_1, &ctrl1, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Control_1 read failed: %s", esp_err_to_name(err));
        s_rtc_dev = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "PCF85063 init OK (Control_1=0x%02X)", ctrl1);
    return true;
}

bool rtcRead(struct tm* time)
{
    if (!s_rtc_dev || !time) return false;

    uint8_t regs[REG_TIME_LENGTH];
    esp_err_t err = rtcReadRegs(REG_TIME_START, regs, REG_TIME_LENGTH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Time read failed: %s", esp_err_to_name(err));
        return false;
    }

    // OS bit (Seconds bit7): 1 = clock integrity not guaranteed
    if (regs[0] & 0x80) {
        ESP_LOGW(TAG, "Clock integrity lost (OS=1), time invalid");
        return false;
    }

    time->tm_sec  = bcdToDec(regs[0] & 0x7F);
    time->tm_min  = bcdToDec(regs[1] & 0x7F);
    time->tm_hour = bcdToDec(regs[2] & 0x3F);
    time->tm_mday = bcdToDec(regs[3] & 0x3F);
    time->tm_wday = regs[4] & 0x07;
    time->tm_mon  = bcdToDec(regs[5] & 0x1F) - 1;  // struct tm: 0-11
    time->tm_year = bcdToDec(regs[6]) + 100;         // struct tm: years since 1900, RTC: 2000 base

    return true;
}

bool rtcWrite(const struct tm* time)
{
    if (!s_rtc_dev || !time) return false;

    uint8_t regs[REG_TIME_LENGTH];
    regs[0] = decToBcd(time->tm_sec);           // Seconds (OS bit cleared)
    regs[1] = decToBcd(time->tm_min);           // Minutes
    regs[2] = decToBcd(time->tm_hour);          // Hours (24h)
    regs[3] = decToBcd(time->tm_mday);          // Days
    regs[4] = time->tm_wday & 0x07;             // Weekdays
    regs[5] = decToBcd(time->tm_mon + 1);       // Months (struct tm: 0-11 → RTC: 1-12)
    regs[6] = decToBcd(time->tm_year - 100);    // Years (struct tm: 1900 base → RTC: 2000 base)

    esp_err_t err = rtcWriteRegs(REG_TIME_START, regs, REG_TIME_LENGTH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Time write failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "RTC set: %04d-%02d-%02d %02d:%02d:%02d UTC",
             time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
             time->tm_hour, time->tm_min, time->tm_sec);
    return true;
}

bool rtcIsReady(void)
{
    return s_rtc_dev != nullptr;
}
