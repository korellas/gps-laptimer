# 하드웨어 사양

**상태:** 2026-02-19 기준 최신
**SSOT:** 하드웨어 사양의 단일 정보원

## 1. 메인 보드

| 구성 요소 | 사양 |
|-----------|------|
| **모델** | Waveshare ESP32-S3-Touch-LCD-3.49 |
| **MCU** | ESP32-S3-WROOM-1 |
| **플래시** | 16 MB |
| **PSRAM** | 8 MB (Octal SPI) |
| **클럭** | 240 MHz (듀얼 코어 Xtensa LX7) |
| **WiFi** | 802.11 b/g/n (SoftAP captive portal, OTA) |
| **Bluetooth** | Bluetooth 5.0 LE (NimBLE, BLE OTA 계획) |

**공식 위키:** https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-3.49

## 2. 디스플레이

| 구성 요소 | 사양 |
|-----------|------|
| **모델** | AXS15231B |
| **인터페이스** | QSPI (4-line SPI) |
| **해상도** | 640×172 픽셀 (네이티브: 172×640 세로) |
| **크기** | 3.49 인치 |
| **색 심도** | RGB565 (16비트) |
| **백라이트** | PWM 제어 |

### 2.1 디스플레이 핀

| 기능 | ESP32-S3 GPIO | 비고 |
|------|---------------|------|
| QSPI_CS | GPIO9 | SPI3_HOST |
| QSPI_SCK | GPIO10 | |
| QSPI_D0 | GPIO11 | |
| QSPI_D1 | GPIO12 | |
| QSPI_D2 | GPIO13 | |
| QSPI_D3 | GPIO14 | |
| RST | GPIO21 | 직접 GPIO 제어, 부팅 시 LOW→HIGH 시퀀스 |
| BL (백라이트) | GPIO8 | PWM 제어 |

> **참고:** RST는 GPIO21로 직접 제어. IO Expander (TCA9554)는 전원 래치(EXIO6)와 모드 선택(EXIO7)을 제어합니다. 섹션 5 참조.

### 2.2 디스플레이 구성

```c
// 패널 네이티브 방향: 172×640 (세로)
// UI 논리적 방향: 640×172 (가로, 90° 회전)

#define PANEL_WIDTH  172
#define PANEL_HEIGHT 640
#define UI_WIDTH     640
#define UI_HEIGHT    172
```

**회전:** 플러시 콜백에서 소프트웨어 회전 (`waveshare_display.cpp`)

## 3. 터치 컨트롤러

| 구성 요소 | 사양 |
|-----------|------|
| **모델** | CST816S |
| **인터페이스** | I2C |
| **I2C 주소** | 0x3B |
| **최대 터치 포인트** | 1 (단일 터치) |

### 3.1 터치 핀

| 기능 | ESP32-S3 GPIO | 비고 |
|------|---------------|------|
| I2C_SDA | GPIO17 | LVGL indev로 통합 |
| I2C_SCL | GPIO18 | |

> **참고:** 터치는 LVGL indev로 통합되어 시작 화면에서 모드 선택, 스와이프 등에 사용됩니다. 11-byte I2C 프로토콜 (`{0xb5,0xab,0xa5,0x5a,...}`)로 좌표를 읽습니다.

## 4. GPS 모듈

| 구성 요소 | 사양 |
|-----------|------|
| **모델** | u-blox G10A-F33 |
| **칩셋** | u-blox M10 (SPG 5.10+) |
| **인터페이스** | UART (3.3V TTL) |
| **프로토콜** | UBX 바이너리 프로토콜 (CFG-VALSET 전용, 레거시 CFG-* 미지원) |
| **업데이트 속도** | 10 Hz (4개 GNSS 동시 사용 시 최대) |
| **채널** | 92 (동시 추적) |
| **GNSS** | GPS, GLONASS, Galileo, BeiDou (4개 동시) |
| **콜드 스타트** | ~24초 |
| **핫 스타트** | ~2초 |

**참조:**
- 데이터시트: https://www.u-blox.com/en/product/max-m10-series
- Integration Manual: UBX-20053088
- Interface Description: UBX-21035062

### 4.1 GPS 핀

| GPS 모듈 핀 | ESP32-S3 GPIO | 기능 | 선 색상 | 비고 |
|-------------|---------------|------|---------|------|
| PPS | GPIO1 | 타임펄스 입력 | 하양 (white) | 정밀 시간 동기화 (섹션 4.4 참조) |
| VCC | 3.3V | 전원 | 빨강 (red) | - |
| TX | GPIO44 | UART2_RX | 파랑 (blue) | GPS TX → ESP32 RX |
| RX | GPIO43 | UART2_TX | 초록 (green) | ESP32 TX → GPS RX |
| GND | GND | 접지 | 검정 (black) | - |
| ENABLE | GPIO0 | 모듈 활성화 | 노랑 (yellow) | 전원 관리용 (섹션 4.5 참조) |

> **참고:** 핀 순서는 GPS 모듈 커넥터 기준 (PPS → VCC → TX → RX → GND → ENABLE)

### 4.2 GPS UART 구성

**초기 보드레이트:** 모듈 출고 시 9600 baud (UBX 기본값)

**운영 보드레이트:** 115200 baud (초기화 시 변경)
- 9600 baud에서는 10Hz NAV-PVT 전송 불가 (100 bytes × 10 = 1000 bytes/s > 960 bytes/s)
- 최소 38400 필요, 115200 권장

```c
// config.h
constexpr int GPS_RX_PIN = 44;  // GPIO44 (RXD) - Connect to GPS TX
constexpr int GPS_TX_PIN = 43;  // GPIO43 (TXD) - Connect to GPS RX

// ublox_gps.h
constexpr int UBLOX_BAUD_INIT   = 9600;    // 모듈 기본값 (초기 연결용)
constexpr int UBLOX_BAUD_TARGET = 115200;   // 운영 보드레이트
```

**초기화 시퀀스:**
1. 9600 baud로 UART 연결
2. UBX-CFG-VALSET로 `CFG-UART1-BAUDRATE = 115200` 전송
3. ESP32 UART를 115200으로 재설정
4. `CFG-RATE-MEAS = 100` (10Hz) 전송
5. NMEA 출력 비활성화 (UBX만 사용)
6. (선택) PPS 10Hz 설정

### 4.3 UBX 설정 시스템 (CFG-VALSET)

M10 시리즈는 **레거시 UBX-CFG-*** 미지원. `UBX-CFG-VALSET` (Class=0x06, ID=0x8A) 전용.

**메시지 구조:**
```
[B5 62] [06 8A] [len_L len_H] [version=0x00] [layers] [00 00] [keyID(4)] [value(N)] ... [CK_A CK_B]
```

**저장 레이어:**

| 레이어 | 비트 | 지속성 | 비고 |
|--------|------|--------|------|
| RAM | 0x01 | 전원 끄면 사라짐 | 즉시 적용 |
| BBR | 0x02 | V_BKUP 연결 시 유지 | 배터리 백업 RAM |
| Flash | 0x04 | 영구 | 항상 유지 |

**운영 방침:** RAM+BBR (0x03)에 저장. 매 부팅 시 재전송으로 V_BKUP 유무와 관계없이 안정 동작 보장.

**Key ID 크기 인코딩** (bits 28-30):
- `0x1_______` = 1 byte (L/bool, U1, E1)
- `0x2_______` = 1 byte (U1, E1)
- `0x3_______` = 2 bytes (U2, I2)
- `0x4_______` = 4 bytes (U4, I4)
- `0x5_______` = 8 bytes (R8)

#### 4.3.1 네비게이션 레이트 설정

| Key | Key ID | 타입 | 설명 |
|-----|--------|------|------|
| `CFG-RATE-MEAS` | `0x30210001` | U2 | 측정 주기 (ms). 100 = 10Hz |
| `CFG-RATE-NAV` | `0x30210002` | U2 | 측정당 네비 솔루션 수 (보통 1) |
| `CFG-RATE-TIMEREF` | `0x20210003` | E1 | 시간 기준 (0=UTC, 1=GPS) |

**최대 네비게이션 레이트 (GNSS 조합별):**

| GNSS 구성 | 최대 Hz |
|-----------|---------|
| GPS 단독 | 25 Hz |
| GPS + Galileo | 20 Hz |
| GPS + Galileo + GLONASS | 16 Hz |
| **GPS + Galileo + GLONASS + BeiDou (전체)** | **10 Hz** ← 사용 |

#### 4.3.2 UART 설정

| Key | Key ID | 타입 | 설명 |
|-----|--------|------|------|
| `CFG-UART1-BAUDRATE` | `0x40520001` | U4 | UART1 보드레이트 |
| `CFG-UART1OUTPROT-UBX` | `0x10740001` | L | UBX 출력 활성화 |
| `CFG-UART1OUTPROT-NMEA` | `0x10740002` | L | NMEA 출력 비활성화 (false) |
| `CFG-UART1INPROT-UBX` | `0x10730001` | L | UBX 입력 활성화 |

#### 4.3.3 메시지 출력 설정

| Key | Key ID | 타입 | 설명 |
|-----|--------|------|------|
| `CFG-MSGOUT-UBX_NAV_PVT_UART1` | `0x20910007` | U1 | NAV-PVT 출력 레이트 (1 = 매 epoch) |

### 4.4 PPS (Pulse Per Second) 핀

**기본 동작:** GNSS 잠금 시 1Hz 펄스 출력

**설정 가능 항목:** 주파수, 듀티사이클, 극성, 잠금/비잠금 별도 설정

| Key | Key ID | 타입 | 설명 |
|-----|--------|------|------|
| `CFG-TP-TP1_ENA` | `0x10050007` | L | 타임펄스 활성화 |
| `CFG-TP-FREQ_LOCK_TP1` | `0x40050025` | U4 | GNSS 잠금 시 주파수 (Hz) |
| `CFG-TP-PERIOD_LOCK_TP1` | `0x40050003` | U4 | GNSS 잠금 시 주기 (μs) |
| `CFG-TP-DUTY_LOCK_TP1` | `0x5005002B` | R8 | GNSS 잠금 시 듀티사이클 (0.0~1.0) |
| `CFG-TP-POL_TP1` | `0x1005000B` | L | 극성 (true = rising edge) |
| `CFG-TP-ALIGN_TO_TOW_TP1` | `0x1005000A` | L | TOW(Top of Week) 정렬 |
| `CFG-TP-SYNC_GNSS_TP1` | `0x10050008` | L | GNSS 시간 동기화 |

**⚠️ 주의:** TIMEPULSE 핀은 **SAFEBOOT_N과 공유**. 부팅 시 LOW이면 세이프부트 모드 진입. 외부 풀다운 회로 금지.

**할당 GPIO:** GPIO1

**현재 계획:** 10Hz PPS로 설정하여 NAV-PVT와 동기화. ESP32 GPIO 인터럽트로 정밀 타이밍 보정에 활용 가능. 당장 필수는 아님 (NAV-PVT의 iTOW로 충분).

### 4.5 EXTINT (외부 인터럽트/전원 관리) 핀

**기본 동작:** 비활성 (floating/open 시 무시)

**용도:**
- 호스트 제어 on/off (GPS 슬립/웨이크)
- Power Save Mode On/Off (PSMOO) 웨이크업

| Key | Key ID | 타입 | 설명 |
|-----|--------|------|------|
| `CFG-PM-OPERATEMODE` | `0x20D00001` | E1 | 0=연속, 1=PSMOO, 2=PSMCT |
| `CFG-PM-EXTINTWAKE` | `0x10D0000C` | L | EXTINT 웨이크업 활성화 |
| `CFG-PM-EXTINTBACKUP` | `0x10D0000D` | L | EXTINT 백업 모드 활성화 |

**전원 모드:**

| 모드 | 설명 | 적합한 용도 |
|------|------|-------------|
| Continuous (0) | 전력 절약 없음, 최대 성능 | **랩타이머 주행 중** ← 사용 |
| PSMOO (1) | 주기적 on/off, RAM 소거됨 | 대기 모드 (>10초 간격) |
| PSMCT (2) | 경량 슬립, 1-2Hz 지원 | 저전력 추적 |

**할당 GPIO:** GPIO0 (ENABLE)

**현재 계획:** 주행 중 Continuous(0) 유지. 향후 배터리 관리 연동 시 ENABLE(EXTINT)로 GPS 슬립 제어 가능.

### 4.6 UBX 프로토콜 상세

**수신 메시지:**

| 메시지 | Class | ID | 페이로드 | 설명 |
|--------|-------|----|----------|------|
| NAV-PVT | 0x01 | 0x07 | 92 bytes | 위치, 속도, 시간, fix 상태 |

**송신 메시지:**

| 메시지 | Class | ID | 설명 |
|--------|-------|----|------|
| CFG-VALSET | 0x06 | 0x8A | 설정값 쓰기 |
| CFG-VALGET | 0x06 | 0x8B | 설정값 읽기 (디버그용) |

**체크섬:** Fletcher-8 (Class부터 Payload 끝까지)

## 5. 센서 I2C 버스 & IO Expander

### 5.1 I2C 버스

| 매개변수 | 값 |
|----------|-----|
| **SDA** | GPIO47 |
| **SCL** | GPIO48 |
| **속도** | 400 kHz (권장) |

**이 버스에 연결된 디바이스:**

| 디바이스 | I2C 주소 | 용도 |
|----------|----------|------|
| TCA9554 IO Expander | 0x20 | GPIO 확장 (INT, 전원 제어) |
| QMI8658C IMU | 0x6B | 6축 가속도/자이로 (섹션 6 참조) |
| PCF85063 RTC | 0x51 | 실시간 시계 (섹션 7 참조) |

> **참고:** 터치 컨트롤러 (CST816S, 0x3B)는 별도 I2C 버스 (GPIO17/18) 사용.

### 5.2 IO Expander (TCA9554)

| 구성 요소 | 사양 |
|-----------|------|
| **모델** | TCA9554 |
| **I2C 주소** | 0x20 (A2=A1=A0=0) |
| **핀 수** | 8 (EXIO0~EXIO7) |

**EXIO 핀 매핑:**

| EXIO 핀 | 회로도 명칭 | 연결 대상 | 방향 | 코드 사용 | 비고 |
|---------|-----------|----------|------|----------|------|
| EXIO0 | TP INT | 터치 인터럽트 | Input | 미사용 | 터치는 I2C 폴링 방식 |
| EXIO1 | BL EN | 백라이트 Enable | Output | 미사용 | 기본 상태로 동작, GPIO8 PWM으로 밝기 제어 |
| EXIO2 | IMU INT1 | QMI8658C INT1 | Input | 미사용 | IMU 인터럽트 1 |
| EXIO3 | IMU INT2 | QMI8658C INT2 | Input | 미사용 | IMU 인터럽트 2 |
| EXIO4 | RTC INT | PCF85063 INT | Input | 미사용 | RTC 인터럽트 |
| EXIO5 | LCD TE | LCD Tearing Effect | Input | 미사용 | 프레임 동기화 신호 (LCD→호스트) |
| EXIO6 | SYS EN | 시스템 전원 래치 | Output | **사용** | HIGH=전원 유지, LOW=전원 차단 |
| EXIO7 | NS MODE | Normal/Standby 모드 | Output | **사용** | HIGH=Normal 모드 |

> **⚠️ 중요:** IMU/RTC의 INT 핀을 읽으려면 TCA9554를 먼저 초기화해야 합니다.

## 6. IMU (QMI8658C)

**상태:** 통합 완료 (v1.1.0)

| 구성 요소 | 사양 |
|-----------|------|
| **모델** | QMI8658C |
| **인터페이스** | I2C (센서 버스, 섹션 5.1 참조) |
| **I2C 주소** | 0x6B |
| **가속도계** | 3축, ±2/±4/±8/±16 g |
| **자이로스코프** | 3축, ±16/±32/±64/±128/±256/±512/±1024/±2048 °/s |
| **출력 데이터 레이트** | 최대 8 kHz (가속도), 최대 8 kHz (자이로) |
| **INT1** | EXIO2 (TCA9554 경유) |
| **INT2** | EXIO3 (TCA9554 경유) |

**참조:**
- 데이터시트: https://files.waveshare.com/wiki/common/QMI8658C_datasheet_rev_0.9.pdf

### 6.1 현재 활용

- **Sensor Fusion:** GPS velNED 차분 vs IMU 가속도 → Wahba's problem (SVD) → 회전 행렬 R (sensor→NED)
- **Speed Kalman Filter:** predict@100Hz (IMU fwdAccel), update@10Hz (GPS 속도) → GPS 단절 시 터널 속도 추정
- **캘리브레이션 저장:** `/spiffs/config/imu_fusion.json` (부팅 시 재사용)
- **드라이버:** `components/imu/qmi8658c.cpp` — 100Hz 가속도+자이로, ±8g / ±512°/s
- **AppContext 통합:** `gApp.imuData`, `gApp.imuCalibration`, `gApp.imuReady`, `gApp.fusedSpeedKmh` 등

> **참고:** IMU는 `imuTask` (Core 1, 100Hz)에서 읽고 Kalman predict. GPS 수신 시에는 GPS 속도를 직접 사용, GPS 단절 시에만 융합 속도 활용.

## 7. RTC (PCF85063)

**상태:** 드라이버 구현 완료 (v1.0.0), GPS 시간 동기화 연동

| 구성 요소 | 사양 |
|-----------|------|
| **모델** | PCF85063 |
| **인터페이스** | I2C (센서 버스, 섹션 5.1 참조) |
| **I2C 주소** | 0x51 |
| **기능** | 시간/날짜 유지, 알람, 타이머 |
| **백업 전원** | 배터리로 전원 꺼져도 시간 유지 |
| **정밀도** | ±2 ppm (약 ±1분/년) |
| **INT** | EXIO4 (TCA9554 경유) |

**참조:**
- 데이터시트: https://files.waveshare.com/wiki/common/Pcf85063atl1118-NdPQpTGE-loeW7GbZ7.pdf

### 7.1 레지스터 구조

| 레지스터 | 주소 | 설명 |
|---------|------|------|
| Control_1 | 0x00 | 운영 모드 제어 |
| Control_2 | 0x01 | 인터럽트, 알람 제어 |
| Seconds | 0x04 | BCD 인코딩, bit 7 = OS (클럭 무결성) |
| Minutes | 0x05 | BCD |
| Hours | 0x06 | BCD (24시간) |
| Days | 0x07 | BCD |
| Weekdays | 0x08 | 0=일요일 |
| Months | 0x09 | BCD |
| Years | 0x0A | BCD (0-99) |

> **참고:** 시간 레지스터는 0x04부터 7바이트 연속 읽기. BCD→10진 변환 필요.

### 7.2 현재 활용

- **GPS 시간 동기화:** GPS fix 시 `settimeofday()`로 시스템 시계 설정 (최초 1회, `gApp.gpsTimeSet`)
- **Status Bar 시계:** KST(UTC+9) 12시간제 표시, GPS 동기화 전 `--:--`
- **드라이버:** `components/rtc/pcf85063.cpp`

## 8. SD 카드

**상태:** 통합 완료 (v1.0.0)

| 구성 요소 | 사양 |
|-----------|------|
| **슬롯** | TF (microSD) 카드 슬롯 |
| **인터페이스** | SDMMC 1-bit 모드 |
| **파일시스템** | FAT32 |
| **마운트 포인트** | `/sdcard` |

### 8.1 SD 카드 핀

| 신호 | ESP32-S3 GPIO | 비고 |
|------|---------------|------|
| D0 | GPIO40 | 데이터 라인 |
| CLK | GPIO41 | 클럭 |
| CMD | GPIO39 | 커맨드 |

### 8.2 SD 카드 구성

```c
// SDMMC 1-bit 모드
// 최대 파일 동시 열기: 5
// 할당 단위 크기: 48 KB (16 * 1024 * 3)
// 미포맷 카드 자동 포맷: 지원 (format_if_mount_failed)
```

### 8.3 현재 활용

- **GPS 트랙 로깅:** 세션 CSV + 이벤트 로그 (`components/sd_logger/`)
- **랩 데이터 저장:** SD 마운트 시 SPIFFS 대신 SD에 우선 저장 (`/sdcard/laps/`)
- **디스플레이 설정:** `/sdcard/display_config.json` (선택)
- **드라이버:** `components/sdcard/sdcard_manager.cpp` (SDMMC 1-bit FAT)

> **참고:** SD 카드 없어도 동작 (SPIFFS fallback). `gApp.sdCardMounted`로 상태 확인.

## 9. 내부 저장소

| 구성 요소 | 사양 |
|-----------|------|
| **유형** | SPIFFS (SPI Flash File System) |
| **파티션 크기** | 2 MB (`partitions.csv` `storage` 파티션) |
| **마운트 포인트** | `/spiffs` |
| **사용 사례** | 랩 데이터 저장, IMU 캘리브레이션, 설정 |

### 9.1 파티션 테이블

프로젝트 루트의 `partitions.csv` 참조.

**현재 레이아웃 (OTA 미지원):**
```csv
# Name,   Type, SubType, Offset,   Size
nvs,      data, nvs,     0x9000,   0x6000
phy_init, data, phy,     0xf000,   0x1000
factory,  app,  factory, 0x10000,  6M
storage,  data, spiffs,  ,         2M
```

**OTA 지원 레이아웃 (적용):**
```csv
# Name,     Type, SubType, Offset,    Size
nvs,        data, nvs,     0x9000,    0x6000
otadata,    data, ota,     0xF000,    0x2000
phy_init,   data, phy,     0x11000,   0x1000
ota_0,      app,  ota_0,   0x20000,   0x600000   # 6MB
ota_1,      app,  ota_1,   0x620000,  0x600000   # 6MB
storage,    data, spiffs,  0xC20000,  0x200000   # 2MB
```

> **참고:** OTA 파티션 전환 시 1회 전체 플래시 필요 (SPIFFS 데이터 소실).
> 상세 설계: [../planning/specs/ota-design.md](../planning/specs/ota-design.md) 섹션 2 참조.

## 10. 전원 및 배터리

| 매개변수 | 값 |
|----------|-----|
| **입력 전압** | 5V (USB-C) |
| **배터리 지원** | 내장 배터리 관리 |
| **배터리 ADC** | GPIO4 (IO4) - 배터리 전압 모니터링 |
| **USB 인터페이스** | USB Serial/JTAG (ESP32-S3 내장) |
| **소비 전류** | ~150-300 mA (디스플레이 켜짐, GPS 활성) |

### 10.1 배터리 모니터링

| 매개변수 | 값 |
|----------|-----|
| **ADC 채널** | ADC1_CHANNEL_3 (GPIO4) |
| **전압 범위** | 0-3.3V (전압 분배기 사용) |
| **배터리 유형** | Li-ion/Li-Po (3.0-4.2V 가정) |
| **ADC 감쇠** | ADC_ATTEN_DB_12 (0-3.3V, ESP-IDF 5.x 명칭) |

**구성:**
```c
// config.h
constexpr float BATTERY_DIVIDER_FACTOR = 3.0f;  // 3:1 전압 분배기
// ADC 읽기: ADC_ATTEN_DB_12, 12-bit, 13회 오버샘플링 + 상하위 25% 트리밍
```

**측정 방식 (코드 기준):**
- 13회 오버샘플링 + 상하위 25% 트리밍 + 중간값 평균
- 21-entry LiPo 방전 곡선 LUT + 선형 보간 → SoC% 계산
- EWMA 방전율 추적 (alpha=0.05) → `gApp.batteryPercent`에 반영

## 11. 오디오 (마이크 및 스피커)

**상태:** 하드웨어 존재, 소프트웨어 미통합

| 구성 요소 | 사양 |
|-----------|------|
| **출력 코덱** | ES8311 (DAC, 스피커) |
| **입력 코덱** | ES7210 (ADC, 마이크) |
| **인터페이스** | I2S (디지털 오디오) |
| **샘플 레이트** | 최대 44.1 kHz |

### 11.1 오디오 핀

| 기능 | ESP32-S3 GPIO | 비고 |
|------|---------------|------|
| I2S_MCLK | GPIO7 | 마스터 클럭 |
| I2S_BCLK | GPIO15 | 비트 클럭 |
| I2S_WS | GPIO46 | 워드 선택 (L/R 클럭) |
| I2S_DOUT | GPIO45 | ESP32→코덱 (스피커), 회로도: DSDIN |
| I2S_DIN | GPIO6 | 코덱→ESP32 (마이크), 회로도: DSOUT |

### 11.2 잠재적 사용 사례

- **오디오 피드백:** 랩 완료, 섹터 전환 시 비프음
- **음성 명령:** 제한된 어휘 (시작, 리셋 등)
- **음성 녹음:** 세션 중 드라이버 메모
- **주변 녹음:** 분석을 위한 엔진 소리

**참조:**
- 오디오 사양은 보드 위키 참조
- ESP-IDF I2S 드라이버 문서

## 12. 디버그 인터페이스

| 인터페이스 | GPIO | 비고 |
|-----------|------|------|
| **USB Serial/JTAG** | 내장 | 기본 콘솔 및 디버그 인터페이스 |
| **UART0** | GPIO43/44 | GPS가 사용 (UART2로 재할당) |

**콘솔 구성:**
- 기본값: USB Serial/JTAG (VFS stdin/stdout)
- 모니터링: `idf.py monitor` (USB CDC ACM)
- 보드레이트: 해당 없음 (USB)

## 13. 환경 사양

| 매개변수 | 값 |
|----------|-----|
| **작동 온도** | -10°C ~ 60°C |
| **보관 온도** | -20°C ~ 70°C |
| **습도** | 10% ~ 90% RH (비응축) |

## 14. 기계적 사양

| 매개변수 | 값 |
|----------|-----|
| **보드 크기** | 82mm × 35mm × 10mm |
| **디스플레이 영역** | 86.76mm × 23.33mm |
| **마운팅** | M2.5 구멍 4개 |

## 15. 핀 할당 요약

| 주변장치 | GPIO 핀 | 비고 |
|---------|---------|------|
| Display QSPI | 9(CS), 10(SCK), 11-14(D0-D3), 8(BL) | AXS15231B, SPI3_HOST |
| 센서 I2C 버스 | 47(SDA), 48(SCL) | 400kHz, TCA9554/QMI8658C/PCF85063 공유 |
| IO Expander | (센서 I2C 버스) | TCA9554, I2C 0x20, EXIO0~7 |
| IMU | (센서 I2C 버스) | QMI8658C, I2C 0x6B, INT→EXIO2/3 |
| RTC | (센서 I2C 버스) | PCF85063, I2C 0x51, INT→EXIO4 |
| Touch I2C | 17(SDA), 18(SCL) | CST816S, I2C 0x3B, LVGL indev |
| GPS UART | 43(TX), 44(RX) | u-blox G10A-F33 |
| GPS PPS | 1 | 타임펄스 (SAFEBOOT_N 공유 주의) |
| GPS ENABLE | 0 | 모듈 활성화 제어 |
| SD Card | 39(CMD), 40(D0), 41(CLK) | SDMMC 1-bit, FAT32 |
| Battery ADC | 4 | 배터리 전압 모니터링 (ADC1_CH3) |
| PWR Button | 16 | 2초 롱프레스 → 전원 OFF |
| Audio I2S | 7(MCLK), 15(BCLK), 46(WS), 45(DOUT), 6(DIN) | ES8311+ES7210 |
| USB Console | 내장 | ESP32-S3 USB Serial/JTAG |

## 참조

- **아키텍처:** [architecture.md](architecture.md)
- **빌드 환경:** [build-environment.md](build-environment.md)
- **UI 사양:** [ui-specification.md](ui-specification.md)
