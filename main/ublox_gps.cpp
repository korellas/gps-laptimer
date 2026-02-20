/**
 * @file ublox_gps.cpp
 * @brief u-blox GPS UBX parser, UART transport, and module configuration (ESP-IDF)
 *
 * u-blox M10 (SPG 5.10+) 전용 — CFG-VALSET 기반 설정
 */

#include "ublox_gps.h"
#include "config.h"

#include <cstdio>
#include <cstring>
#include <cmath>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "UBLOX_GPS";

// ============================================================
// MODULE STATE
// ============================================================

static constexpr int kUartNum = UART_NUM_2;
static int s_rxPin = -1;
static int s_txPin = -1;
static bool s_uart_started = false;
static UBloxData lastData;
static uint8_t buffer[600];  // NAV-SAT: 8 + 12×40 SVs + 프레임 오버헤드 = ~500
static uint16_t bufIndex = 0;

// Phase 2: 새 NAV-PVT 프레임 파싱 완료 플래그
static bool s_newFrameParsed = false;

// 통계 및 로깅
static uint32_t s_checksumOk = 0;
static uint32_t s_checksumFail = 0;
static uint32_t s_navPvtCount = 0;           // 1초 윈도우 내 NAV-PVT 수신 횟수
static int64_t s_hzWindowStartUs = 0;        // Hz 측정 윈도우 시작 (us)
static float s_measuredHz = 0.0f;            // 마지막 측정된 Hz
static uint32_t s_totalBytesRead = 0;        // UART에서 읽은 총 바이트 수
static int64_t s_lastPeriodicLogUs = 0;      // 마지막 주기 로그 시각
static uint8_t s_lastLoggedFixType = 0xFF;   // 변경 감지용
static uint8_t s_lastLoggedSats = 0xFF;      // 변경 감지용

// GPS 모듈 활성화 상태
static bool s_moduleEnabled = false;

// NAV-SAT 위성 상세 데이터
static NavSatData s_navSatData;

// ACK 추적
static bool s_ackReceived = false;
static bool s_nakReceived = false;
static uint8_t s_ackClass = 0;
static uint8_t s_ackId = 0;

// ============================================================
// UBX CHECKSUM (Fletcher-8)
// ============================================================

static void calcUbxChecksum(const uint8_t* data, uint16_t len, uint8_t& ckA, uint8_t& ckB) {
    ckA = 0;
    ckB = 0;
    for (uint16_t i = 0; i < len; i++) {
        ckA += data[i];
        ckB += ckA;
    }
}

// ============================================================
// UBX MESSAGE PARSER
// ============================================================

static bool parseUbxMessage(const uint8_t* data, uint16_t len) {
    if (len < 8) return false;

    if (data[0] != UBX_SYNC1 || data[1] != UBX_SYNC2) return false;

    uint8_t msgClass = data[2];
    uint8_t msgId = data[3];
    uint16_t payloadLen = data[4] | (data[5] << 8);

    // 체크섬 검증
    uint8_t ckA = data[6 + payloadLen];
    uint8_t ckB = data[7 + payloadLen];
    uint8_t calcCkA, calcCkB;
    calcUbxChecksum(data + 2, 4 + payloadLen, calcCkA, calcCkB);

    if (calcCkA != ckA || calcCkB != ckB) {
        s_checksumFail++;
        ESP_LOGW(TAG, "Checksum mismatch (class=0x%02X id=0x%02X, fail#%lu)",
                 msgClass, msgId, (unsigned long)s_checksumFail);
        return false;
    }
    s_checksumOk++;

    const uint8_t* payload = data + 6;

    // ─── NAV-PVT (0x01/0x07) ───
    // UBX-NAV-PVT 표준 오프셋 (u-blox M10 SPG 5.10 Interface Description)
    if (msgClass == UBX_CLASS_NAV && msgId == UBX_ID_NAV_PVT) {
        if (payloadLen < 92) return false;

        // iTOW (offset 0, U4)
        lastData.iTOW = payload[0] | (payload[1] << 8) |
                       (payload[2] << 16) | (payload[3] << 24);

        // UTC 시간 (offset 4~11)
        lastData.year   = payload[4] | (payload[5] << 8);
        lastData.month  = payload[6];
        lastData.day    = payload[7];
        lastData.hour   = payload[8];
        lastData.minute = payload[9];
        lastData.second = payload[10];
        uint8_t validFlags = payload[11];
        lastData.timeValid = ((validFlags & 0x03) == 0x03);

        // Fix type (offset 20, U1)
        lastData.fixType = payload[20];

        // Satellites (offset 23, U1)
        lastData.satellites = payload[23];

        // lon (offset 24, I4, degrees * 1e-7)
        // lat (offset 28, I4, degrees * 1e-7)
        int32_t lonDeg7 = (int32_t)(payload[24] | (payload[25] << 8) |
                          (payload[26] << 16) | (payload[27] << 24));
        int32_t latDeg7 = (int32_t)(payload[28] | (payload[29] << 8) |
                          (payload[30] << 16) | (payload[31] << 24));

        lastData.lon = lonDeg7 / 10000000.0;
        lastData.lat = latDeg7 / 10000000.0;

        // gSpeed (offset 60, I4, mm/s) — 2D ground speed
        int32_t gSpeed = (int32_t)(payload[60] | (payload[61] << 8) |
                         (payload[62] << 16) | (payload[63] << 24));
        lastData.speedKmh = gSpeed * 0.0036f;  // mm/s → km/h

        // headMot (offset 64, I4, degrees * 1e-5) — heading of motion
        int32_t headMot = (int32_t)(payload[64] | (payload[65] << 8) |
                          (payload[66] << 16) | (payload[67] << 24));
        lastData.headingDeg = headMot / 100000.0f;

        // hMSL (offset 32, I4, mm → m) — 해발 고도
        int32_t hMSL_mm = (int32_t)(payload[32] | (payload[33] << 8) |
                          (payload[34] << 16) | (payload[35] << 24));
        lastData.altitudeM = hMSL_mm / 1000.0f;

        // velN/velE/velD (offsets 48/52/56, I4, mm/s) — NED 속도 벡터
        int32_t velN = (int32_t)(payload[48] | (payload[49] << 8) |
                       (payload[50] << 16) | (payload[51] << 24));
        int32_t velE = (int32_t)(payload[52] | (payload[53] << 8) |
                       (payload[54] << 16) | (payload[55] << 24));
        int32_t velD = (int32_t)(payload[56] | (payload[57] << 8) |
                       (payload[58] << 16) | (payload[59] << 24));
        lastData.velNorthMps = velN * 0.001f;  // mm/s → m/s
        lastData.velEastMps  = velE * 0.001f;
        lastData.velDownMps  = velD * 0.001f;

        // pDOP (offset 76, U2, × 0.01)
        uint16_t pDOP_raw = (uint16_t)(payload[76] | (payload[77] << 8));
        lastData.hdop = pDOP_raw * 0.01f;

        // 3D fix 이상만 유효
        lastData.valid = (lastData.fixType >= 3);

        // Phase 2: 새 프레임 파싱 완료 표시
        s_newFrameParsed = true;

        // Hz 측정용 카운터
        s_navPvtCount++;

        // Fix type 또는 위성 수 변경 시 로그
        if (lastData.fixType != s_lastLoggedFixType) {
            const char* fixStr = lastData.fixType == 0 ? "No fix" :
                                 lastData.fixType == 1 ? "Dead reckoning" :
                                 lastData.fixType == 2 ? "2D" :
                                 lastData.fixType == 3 ? "3D" :
                                 lastData.fixType == 4 ? "GNSS+DR" : "Unknown";
            ESP_LOGI(TAG, "Fix changed: %s (type=%d, sats=%d)",
                     fixStr, lastData.fixType, lastData.satellites);
            s_lastLoggedFixType = lastData.fixType;
        }
        if (lastData.satellites != s_lastLoggedSats) {
            ESP_LOGD(TAG, "Satellites: %d (fix=%d)", lastData.satellites, lastData.fixType);
            s_lastLoggedSats = lastData.satellites;
        }

        return true;
    }

    // ─── ACK-ACK (0x05/0x01) ───
    if (msgClass == UBX_CLASS_ACK && msgId == UBX_ID_ACK_ACK) {
        if (payloadLen >= 2) {
            s_ackClass = payload[0];
            s_ackId = payload[1];
            s_ackReceived = true;
            ESP_LOGD(TAG, "ACK for class=0x%02X id=0x%02X", s_ackClass, s_ackId);
        }
        return true;
    }

    // ─── ACK-NAK (0x05/0x00) ───
    if (msgClass == UBX_CLASS_ACK && msgId == UBX_ID_ACK_NAK) {
        if (payloadLen >= 2) {
            s_ackClass = payload[0];
            s_ackId = payload[1];
            s_nakReceived = true;
            ESP_LOGW(TAG, "NAK for class=0x%02X id=0x%02X", s_ackClass, s_ackId);
        }
        return true;
    }

    // ─── NAV-SAT (0x01/0x35) ───
    if (msgClass == UBX_CLASS_NAV && msgId == UBX_ID_NAV_SAT) {
        if (payloadLen < 8) return false;

        uint8_t numSvs = payload[5];
        uint16_t expectedLen = 8 + 12 * (uint16_t)numSvs;
        if (payloadLen < expectedLen) return false;

        NavSatData sat = {};
        float cnoSum = 0.0f;
        int cnoCount = 0;

        for (int i = 0; i < numSvs; i++) {
            const uint8_t* sv = &payload[8 + 12 * i];
            uint8_t gnssId = sv[0];
            uint8_t cno    = sv[2];
            uint32_t flags = sv[8] | (sv[9] << 8) | (sv[10] << 16) | (sv[11] << 24);
            bool svUsed = (flags >> 3) & 0x01;

            // cno > 0 이면 보이는 위성
            if (cno > 0) {
                cnoSum += cno;
                cnoCount++;
            }

            uint8_t* vis = nullptr;
            uint8_t* used = nullptr;
            switch (gnssId) {
                case 0: vis = &sat.visGps; used = &sat.usedGps; break;  // GPS
                case 2: vis = &sat.visGal; used = &sat.usedGal; break;  // Galileo
                case 3: vis = &sat.visBds; used = &sat.usedBds; break;  // BeiDou
                case 6: vis = &sat.visGlo; used = &sat.usedGlo; break;  // GLONASS
                default: continue;  // SBAS, QZSS 등 무시
            }
            if (cno > 0) (*vis)++;
            if (svUsed) (*used)++;
        }

        sat.numSvs = sat.visGps + sat.visGlo + sat.visGal + sat.visBds;
        sat.avgCno = cnoCount > 0 ? cnoSum / cnoCount : 0.0f;
        sat.valid = true;
        s_navSatData = sat;

        return true;
    }

    return false;
}

// ============================================================
// UART STREAM PROCESSOR
// ============================================================

static void processUartStream(uint8_t c) {
    if (bufIndex == 0 && c != UBX_SYNC1) return;
    if (bufIndex == 1 && c != UBX_SYNC2) {
        bufIndex = 0;
        if (c == UBX_SYNC1) bufIndex = 1;
        return;
    }

    buffer[bufIndex++] = c;

    if (bufIndex >= 6) {
        uint16_t payloadLen = buffer[4] | (buffer[5] << 8);
        uint16_t totalLen = 6 + payloadLen + 2;

        if (totalLen > sizeof(buffer)) {
            bufIndex = 0;
            return;
        }

        if (bufIndex >= totalLen) {
            parseUbxMessage(buffer, totalLen);
            bufIndex = 0;
        }
    }

    if (bufIndex >= sizeof(buffer)) {
        bufIndex = 0;
    }
}

// ============================================================
// UART 초기화 / 재초기화
// ============================================================

static bool initUart(int baudRate) {
    // 기존 드라이버가 있으면 제거
    if (s_uart_started) {
        uart_driver_delete((uart_port_t)kUartNum);
        s_uart_started = false;
    }

    uart_config_t cfg = {};
    cfg.baud_rate = baudRate;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    if (uart_param_config((uart_port_t)kUartNum, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed (baud=%d)", baudRate);
        return false;
    }

    if (uart_set_pin((uart_port_t)kUartNum, s_txPin, s_rxPin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "UART pin config failed");
        return false;
    }

    if (uart_driver_install((uart_port_t)kUartNum, 2048, 0, 0, nullptr, 0) != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return false;
    }

    s_uart_started = true;
    bufIndex = 0;
    return true;
}

// ============================================================
// PUBLIC API: 초기화
// ============================================================

void initUBloxGPS(int rxPin, int txPin) {
    ESP_LOGI(TAG, "Initializing GPS...");

    s_rxPin = rxPin;
    s_txPin = txPin;
    lastData = UBloxData();
    s_checksumOk = 0;
    s_checksumFail = 0;
    s_navPvtCount = 0;
    s_hzWindowStartUs = 0;
    s_measuredHz = 0.0f;
    s_totalBytesRead = 0;
    s_lastPeriodicLogUs = 0;
    s_lastLoggedFixType = 0xFF;
    s_lastLoggedSats = 0xFF;

    if (!initUart(UBLOX_BAUD_INIT)) {
        ESP_LOGE(TAG, "UART init failed!");
        return;
    }

    ESP_LOGI(TAG, "UART2 ready: RX=GPIO%d, TX=GPIO%d, Baud=%d", rxPin, txPin, UBLOX_BAUD_INIT);
}

// ============================================================
// PUBLIC API: GPS 데이터 업데이트
// ============================================================

bool updateUBloxGPS() {
    if (!s_uart_started) return false;

    // Phase 2: 새 프레임 플래그 리셋
    s_newFrameParsed = false;

    uint8_t chunk[64];
    int len;
    while ((len = uart_read_bytes((uart_port_t)kUartNum, chunk, sizeof(chunk), 0)) > 0) {
        s_totalBytesRead += len;
        for (int i = 0; i < len; i++) {
            processUartStream(chunk[i]);
        }
    }

    // ─── Hz 측정 (1초 윈도우) ───
    int64_t nowUs = esp_timer_get_time();
    if (s_hzWindowStartUs == 0) {
        s_hzWindowStartUs = nowUs;
    }
    int64_t elapsedUs = nowUs - s_hzWindowStartUs;
    if (elapsedUs >= 1000000) {  // 1초 경과
        s_measuredHz = (float)s_navPvtCount * 1000000.0f / (float)elapsedUs;
        s_navPvtCount = 0;
        s_hzWindowStartUs = nowUs;
    }

    // ─── 주기 로그 (5초마다) ───
    if (s_lastPeriodicLogUs == 0) s_lastPeriodicLogUs = nowUs;
    if ((nowUs - s_lastPeriodicLogUs) >= 5000000) {  // 5초
        s_lastPeriodicLogUs = nowUs;
        uint32_t totalMsgs = s_checksumOk + s_checksumFail;
        int healthPct = totalMsgs > 0 ? (int)(s_checksumOk * 100 / totalMsgs) : 0;

        if (lastData.fixType >= 2) {
            ESP_LOGI(TAG, "[GPS] fix=%d sat=%d rate=%.1fHz pos=(%.6f,%.6f) "
                     "spd=%.1fkm/h hdg=%.1f alt=%.1fm hdop=%.1f "
                     "uart=%luB ok=%lu err=%lu(%d%%)",
                     lastData.fixType, lastData.satellites, s_measuredHz,
                     lastData.lat, lastData.lon,
                     lastData.speedKmh, lastData.headingDeg,
                     lastData.altitudeM, lastData.hdop,
                     (unsigned long)s_totalBytesRead,
                     (unsigned long)s_checksumOk,
                     (unsigned long)s_checksumFail,
                     healthPct);
        } else {
            ESP_LOGI(TAG, "[GPS] fix=%d sat=%d rate=%.1fHz uart=%luB ok=%lu err=%lu(%d%%) — waiting for fix",
                     lastData.fixType, lastData.satellites, s_measuredHz,
                     (unsigned long)s_totalBytesRead,
                     (unsigned long)s_checksumOk,
                     (unsigned long)s_checksumFail,
                     healthPct);
        }

        if (lastData.timeValid) {
            ESP_LOGI(TAG, "[GPS] UTC: %04u-%02u-%02u %02u:%02u:%02u",
                     lastData.year, lastData.month, lastData.day,
                     lastData.hour, lastData.minute, lastData.second);
        }
    }

    return s_newFrameParsed;
}

UBloxData getUBloxData() {
    return lastData;
}

UBloxStats getUBloxStats() {
    return { s_checksumOk, s_checksumFail, s_measuredHz, s_totalBytesRead };
}

NavSatData getNavSatData() {
    return s_navSatData;
}

// ============================================================
// UBX 메시지 전송
// ============================================================

bool sendUbxMessage(uint8_t msgClass, uint8_t msgId,
                    const uint8_t* payload, uint16_t payloadLen) {
    if (!s_uart_started) return false;

    // 프레임 크기: sync(2) + class(1) + id(1) + len(2) + payload + cksum(2)
    uint16_t frameLen = 8 + payloadLen;
    if (frameLen > 512) return false;  // 안전 제한

    uint8_t frame[512];
    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = msgClass;
    frame[3] = msgId;
    frame[4] = payloadLen & 0xFF;
    frame[5] = (payloadLen >> 8) & 0xFF;

    if (payloadLen > 0 && payload != nullptr) {
        memcpy(frame + 6, payload, payloadLen);
    }

    uint8_t ckA, ckB;
    calcUbxChecksum(frame + 2, 4 + payloadLen, ckA, ckB);
    frame[6 + payloadLen] = ckA;
    frame[7 + payloadLen] = ckB;

    int written = uart_write_bytes((uart_port_t)kUartNum, frame, frameLen);
    return (written == frameLen);
}

// ============================================================
// CFG-VALSET 헬퍼
// ============================================================

// CFG-VALSET 키-값 쌍 구조체
struct CfgKeyValue {
    uint32_t key;
    uint32_t value;
    uint8_t size;   // 1(U1/E1), 2(U2), 4(U4)
};

/**
 * @brief CFG-VALSET 메시지 전송
 * @param items 키-값 배열
 * @param count 배열 크기
 * @param layers 저장 레이어 (0x01=RAM, 0x02=BBR, 0x03=RAM+BBR)
 */
static bool sendCfgValset(const CfgKeyValue* items, int count, uint8_t layers) {
    // CFG-VALSET payload: version(1) + layers(1) + reserved(2) + key-value pairs
    uint8_t payload[128];
    int pos = 0;

    payload[pos++] = 0x01;    // version
    payload[pos++] = layers;  // layers
    payload[pos++] = 0x00;    // reserved
    payload[pos++] = 0x00;    // reserved

    for (int i = 0; i < count; i++) {
        // Key ID (4 bytes, little-endian)
        payload[pos++] = (items[i].key >>  0) & 0xFF;
        payload[pos++] = (items[i].key >>  8) & 0xFF;
        payload[pos++] = (items[i].key >> 16) & 0xFF;
        payload[pos++] = (items[i].key >> 24) & 0xFF;

        // Value (little-endian, variable size)
        uint32_t val = items[i].value;
        for (int b = 0; b < items[i].size; b++) {
            payload[pos++] = (val >> (b * 8)) & 0xFF;
        }

        if (pos >= (int)sizeof(payload) - 8) {
            ESP_LOGE(TAG, "CFG-VALSET payload overflow");
            return false;
        }
    }

    return sendUbxMessage(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, pos);
}

// ============================================================
// ACK 대기
// ============================================================

/**
 * @brief UBX-ACK 응답 대기 (polling)
 * @param timeoutMs 타임아웃 (ms)
 * @return true=ACK 수신, false=NAK 또는 타임아웃
 */
static bool waitForAck(uint16_t timeoutMs) {
    s_ackReceived = false;
    s_nakReceived = false;

    unsigned long start = (unsigned long)(esp_timer_get_time() / 1000ULL);
    while (true) {
        unsigned long now = (unsigned long)(esp_timer_get_time() / 1000ULL);
        if ((now - start) > timeoutMs) {
            ESP_LOGW(TAG, "ACK timeout (%dms)", timeoutMs);
            return false;
        }

        // UART 폴링하여 ACK/NAK 수신
        uint8_t chunk[64];
        int len;
        while ((len = uart_read_bytes((uart_port_t)kUartNum, chunk, sizeof(chunk), 0)) > 0) {
            for (int i = 0; i < len; i++) {
                processUartStream(chunk[i]);
            }
        }

        if (s_ackReceived) return true;
        if (s_nakReceived) {
            ESP_LOGW(TAG, "NAK received");
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
// NAV-SAT ON/OFF
// ============================================================

void enableNavSatOutput(bool enable) {
    if (!s_uart_started) return;

    CfgKeyValue item = { 0x20910016, enable ? 1u : 0u, 1 };  // CFG-MSGOUT-UBX_NAV_SAT_UART1
    sendCfgValset(&item, 1, 0x01);  // RAM only
    if (waitForAck(500)) {
        ESP_LOGI(TAG, "NAV-SAT output %s", enable ? "enabled" : "disabled");
    } else {
        ESP_LOGW(TAG, "NAV-SAT output %s failed", enable ? "enable" : "disable");
    }
    if (!enable) {
        s_navSatData = NavSatData();  // 비활성화 시 데이터 초기화
    }
}

// ============================================================
// GPS 모듈 전원 관리
// ============================================================

void enableGPSModule() {
    if (s_moduleEnabled) return;

    ESP_LOGI(TAG, "Enabling GPS module (GPIO%d HIGH)...", GPS_ENABLE_PIN);

    // GPIO0 HIGH → 모듈 전원 ON
    gpio_set_level((gpio_num_t)GPS_ENABLE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));  // 모듈 부팅 대기

    // UART 초기화 + 모듈 설정
    initUBloxGPS(GPS_RX_PIN, GPS_TX_PIN);
    configureUBloxModule();

    s_moduleEnabled = true;
    ESP_LOGI(TAG, "GPS module enabled");
}

void disableGPSModule() {
    ESP_LOGI(TAG, "Disabling GPS module (GPIO%d LOW)", GPS_ENABLE_PIN);
    gpio_set_level((gpio_num_t)GPS_ENABLE_PIN, 0);
    s_moduleEnabled = false;
}

bool isGPSModuleEnabled() {
    return s_moduleEnabled;
}

// ============================================================
// 모듈 설정 (부트 시퀀스)
// ============================================================

bool configureUBloxModule() {
    ESP_LOGI(TAG, "Configuring u-blox module...");

    // ─── Step 1: Baud rate 확인/변경 ───
    // 먼저 115200에서 데이터 수신 시도 (warm reboot case)
    ESP_LOGI(TAG, "  Checking if module already at %d...", UBLOX_BAUD_TARGET);
    if (initUart(UBLOX_BAUD_TARGET)) {
        vTaskDelay(pdMS_TO_TICKS(200));

        // 200ms 동안 유효 데이터 확인
        s_newFrameParsed = false;
        uint8_t chunk[64];
        int len;
        while ((len = uart_read_bytes((uart_port_t)kUartNum, chunk, sizeof(chunk), 0)) > 0) {
            for (int i = 0; i < len; i++) {
                processUartStream(chunk[i]);
            }
        }

        if (s_newFrameParsed || s_checksumOk > 0) {
            ESP_LOGI(TAG, "  Module already at %d baud (warm reboot)", UBLOX_BAUD_TARGET);
            // Skip baud change, go to rate config
            goto configure_rate;
        }
    }

    // ─── 9600 baud에서 시작 (cold boot) ───
    ESP_LOGI(TAG, "  Starting from %d baud (cold boot)...", UBLOX_BAUD_INIT);
    if (!initUart(UBLOX_BAUD_INIT)) {
        ESP_LOGE(TAG, "  UART init at %d failed", UBLOX_BAUD_INIT);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    {
        // CFG-UART1-BAUDRATE (key 0x40520001, U4)
        CfgKeyValue baudCfg[] = {
            { 0x40520001, UBLOX_BAUD_TARGET, 4 },
        };

        ESP_LOGI(TAG, "  Setting baud rate to %d...", UBLOX_BAUD_TARGET);
        if (!sendCfgValset(baudCfg, 1, 0x03)) {
            ESP_LOGE(TAG, "  Failed to send baud rate config");
            return false;
        }
    }

    // 모듈이 baud rate 변경 처리할 시간
    vTaskDelay(pdMS_TO_TICKS(100));

    // UART RX 버퍼 flush
    uart_flush_input((uart_port_t)kUartNum);
    bufIndex = 0;

    // ─── ESP32 UART를 115200으로 재초기화 ───
    ESP_LOGI(TAG, "  Reinitializing UART at %d...", UBLOX_BAUD_TARGET);
    if (!initUart(UBLOX_BAUD_TARGET)) {
        ESP_LOGE(TAG, "  UART reinit failed, falling back to %d", UBLOX_BAUD_INIT);
        initUart(UBLOX_BAUD_INIT);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

configure_rate:
    // ─── 10Hz + Automotive 설정 (재시도 포함) ───
    {
        CfgKeyValue rateCfg[] = {
            { 0x30210001, 100, 2 },   // RATE-MEAS = 100ms (10Hz)
            { 0x30210002, 1,   2 },   // RATE-NAV = 1
            { 0x20110021, 4,   1 },   // DYNMODEL = Automotive
        };

        bool rateConfigured = false;
        for (int attempt = 0; attempt < 3; attempt++) {
            ESP_LOGI(TAG, "  Setting 10Hz rate + Automotive mode (attempt %d)...", attempt + 1);
            if (sendCfgValset(rateCfg, 3, 0x03)) {
                if (waitForAck(1000)) {
                    ESP_LOGI(TAG, "  Configuration ACK received");
                    rateConfigured = true;
                    break;
                } else {
                    ESP_LOGW(TAG, "  No ACK on attempt %d", attempt + 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
        }

        if (!rateConfigured) {
            ESP_LOGW(TAG, "  Rate config not ACKed after 3 attempts (module may still work)");
        }
    }

    // ─── GNSS 컨스텔레이션 설정: GPS + GLO + GAL + BDS (전체 활성) ───
    {
        CfgKeyValue gnssCfg[] = {
            { 0x1031001F, 1, 1 },   // CFG-SIGNAL-GPS_ENA = enable
            { 0x10310021, 1, 1 },   // CFG-SIGNAL-GAL_ENA = enable
            { 0x10310025, 1, 1 },   // CFG-SIGNAL-GLO_ENA = enable
            { 0x10310022, 1, 1 },   // CFG-SIGNAL-BDS_ENA = enable
        };

        ESP_LOGI(TAG, "  Setting GNSS: GPS+GAL+GLO+BDS (all enabled)...");
        if (sendCfgValset(gnssCfg, 4, 0x03)) {
            if (waitForAck(1000)) {
                ESP_LOGI(TAG, "  GNSS config ACK received");
            } else {
                ESP_LOGW(TAG, "  GNSS config no ACK (module may still apply)");
            }
        }
    }

    // ─── NMEA 출력 비활성화 (UBX만 사용, UART 대역폭 확보) ───
    {
        CfgKeyValue protCfg[] = {
            { 0x10740001, 1, 1 },   // CFG-UART1OUTPROT-UBX = enable
            { 0x10740002, 0, 1 },   // CFG-UART1OUTPROT-NMEA = disable
        };

        ESP_LOGI(TAG, "  Disabling NMEA output (UBX only)...");
        if (sendCfgValset(protCfg, 2, 0x03)) {
            if (waitForAck(1000)) {
                ESP_LOGI(TAG, "  NMEA disabled, UBX-only mode");
            } else {
                ESP_LOGW(TAG, "  NMEA config no ACK");
            }
        }
    }

    ESP_LOGI(TAG, "GPS module configured: %d baud, 10Hz, Automotive, GPS+GAL+GLO+BDS, UBX-only", UBLOX_BAUD_TARGET);
    return true;
}
