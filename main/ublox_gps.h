/**
 * @file ublox_gps.h
 * @brief u-blox GPS 모듈 드라이버 (UBX 프로토콜)
 */

#ifndef UBLOX_GPS_H
#define UBLOX_GPS_H

#include <cstdint>

// ============================================================
// UBX 프로토콜
// ============================================================

// UBX Header
constexpr uint8_t UBX_SYNC1 = 0xB5;
constexpr uint8_t UBX_SYNC2 = 0x62;

// UBX Class
constexpr uint8_t UBX_CLASS_NAV = 0x01;  // Navigation results
constexpr uint8_t UBX_CLASS_ACK = 0x05;  // ACK/NAK
constexpr uint8_t UBX_CLASS_CFG = 0x06;  // Configuration

// UBX IDs
constexpr uint8_t UBX_ID_NAV_PVT    = 0x07;  // Position, Velocity, Time, Fix
constexpr uint8_t UBX_ID_ACK_ACK    = 0x01;  // ACK 성공
constexpr uint8_t UBX_ID_ACK_NAK    = 0x00;  // ACK 실패
constexpr uint8_t UBX_ID_NAV_SAT    = 0x35;  // Satellite information
constexpr uint8_t UBX_ID_CFG_VALSET = 0x8A;  // Configuration value set (M10+)

// GPS 모듈 설정
constexpr int UBLOX_BAUD_INIT   = 9600;    // 초기 연결용 (공장 기본값)
constexpr int UBLOX_BAUD_TARGET = 115200;   // 운용 baud rate (10Hz 지원)

// ============================================================
// 데이터 구조체
// ============================================================

struct UBloxData {
    double lat;           // 위도 (degrees)
    double lon;           // 경도 (degrees)
    float speedKmh;       // 속도 (km/h)
    float headingDeg;     // 헤딩 (degrees)
    uint8_t fixType;      // Fix type: 0=No fix, 1=Dead reckoning, 2=2D, 3=3D, 4=GNSS+DR
    uint8_t satellites;   // 위성 수
    bool valid;           // 데이터 유효 여부
    uint32_t iTOW;        // GPS Time of Week (ms)

    // UTC 시간 (NAV-PVT offset 4~11)
    uint16_t year;        // UTC year
    uint8_t month;        // UTC month (1-12)
    uint8_t day;          // UTC day (1-31)
    uint8_t hour;         // UTC hour (0-23)
    uint8_t minute;       // UTC minute (0-59)
    uint8_t second;       // UTC second (0-59)
    bool timeValid;       // validDate && validTime (offset 11, bit 0+1)

    // 추가 센서 데이터
    float altitudeM;      // 해발 고도 (hMSL, offset 32-35, mm→m)
    float hdop;           // pDOP (offset 76-77, ×0.01)

    // NED 속도 벡터 (센서 퓨전 캘리브레이션용)
    float velNorthMps;    // 북쪽 속도 (m/s, offset 48-51)
    float velEastMps;     // 동쪽 속도 (m/s, offset 52-55)
    float velDownMps;     // 하강 속도 (m/s, offset 56-59, 양수=하강)

    UBloxData() : lat(0), lon(0), speedKmh(0), headingDeg(0),
                  fixType(0), satellites(0), valid(false), iTOW(0),
                  year(0), month(0), day(0), hour(0), minute(0), second(0),
                  timeValid(false), altitudeM(0), hdop(99.9f),
                  velNorthMps(0), velEastMps(0), velDownMps(0) {}
};

// GPS 통계 (Hz 측정, UART 건강도)
struct UBloxStats {
    uint32_t checksumOk;
    uint32_t checksumFail;
    float measuredHz;       // NAV-PVT 수신 빈도 (1초 윈도우)
    uint32_t totalBytes;    // UART에서 읽은 총 바이트 수
};

// GNSS 시스템별 위성 정보 (NAV-SAT)
struct NavSatData {
    uint8_t visGps, visGlo, visGal, visBds;     // 보이는 위성 수
    uint8_t usedGps, usedGlo, usedGal, usedBds; // Fix에 사용 중인 위성 수
    uint8_t numSvs;        // 전체 보이는 위성 수
    float avgCno;          // 평균 신호 강도 (dBHz)
    bool valid;            // NAV-SAT 데이터 수신 여부

    NavSatData() : visGps(0), visGlo(0), visGal(0), visBds(0),
                   usedGps(0), usedGlo(0), usedGal(0), usedBds(0),
                   numSvs(0), avgCno(0), valid(false) {}
};

// ============================================================
//_PUBLIC API
// ============================================================

/**
 * @brief GPS 초기화
 * @param rxPin ESP32 RX 핀 번호
 * @param txPin ESP32 TX 핀 번호
 */
void initUBloxGPS(int rxPin, int txPin);

/**
 * @brief GPS 데이터 업데이트
 * @return 새로운 데이터 수신 여부
 */
bool updateUBloxGPS();

/**
 * @brief 마지막 유효 데이터 가져오기
 * @return 마지막으로 파싱된 GPS 데이터
 */
UBloxData getUBloxData();

/**
 * @brief UBX 메시지 전송
 * @return 전송 성공 여부
 */
bool sendUbxMessage(uint8_t msgClass, uint8_t msgId,
                    const uint8_t* payload, uint16_t payloadLen);

/**
 * @brief u-blox M10 모듈 설정 (부트 시 호출)
 *
 * 9600 baud로 시작 → 115200으로 전환 → 10Hz 설정
 * @return 설정 성공 여부
 */
bool configureUBloxModule();

/**
 * @brief GPS 통계 가져오기 (Hz 측정, UART 건강도)
 */
UBloxStats getUBloxStats();

/**
 * @brief GPS 모듈 전원 ON + UART 초기화 + 모듈 설정
 * 이미 활성화된 경우 no-op
 */
void enableGPSModule();

/**
 * @brief GPS 모듈 전원 OFF (GPIO0 LOW)
 */
void disableGPSModule();

/**
 * @brief GPS 모듈 활성화 여부
 */
bool isGPSModuleEnabled();

/**
 * @brief NAV-SAT 위성 상세 정보 가져오기
 */
NavSatData getNavSatData();

/**
 * @brief NAV-SAT 메시지 출력 on/off (GPS Status 페이지용)
 * @param enable true=활성화, false=비활성화
 */
void enableNavSatOutput(bool enable);

#endif // UBLOX_GPS_H
