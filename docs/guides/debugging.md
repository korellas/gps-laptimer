# 디버깅 가이드

**최종 업데이트:** 2026-02-16

## 1. 시리얼 모니터

ESP-IDF 시리얼 모니터로 실시간 로그 확인:

```bash
idf.py -p COM3 monitor
```

**유용한 시리얼 명령:**

| 명령 | 동작 |
|------|------|
| `s` | 현재 상태 출력 (모드, 랩, GPS, 배터리) |
| `e` | GPS ↔ 시뮬레이션 모드 토글 |
| `r` | 현재 랩 리셋 |
| `v` | 디버그 출력 토글 (`DEBUG_OUTPUT`) |
| `w` | WiFi ON/OFF 토글 |
| `h` | 도움말 |

## 2. 로그 레벨

ESP-IDF 로그 시스템 사용 (`ESP_LOGx`):

```cpp
ESP_LOGE(TAG, "Error: ...");   // 에러 (항상 표시)
ESP_LOGW(TAG, "Warning: ...");  // 경고
ESP_LOGI(TAG, "Info: ...");     // 정보 (기본 레벨)
ESP_LOGD(TAG, "Debug: ...");    // 디버그 (sdkconfig에서 활성화)
```

**개발 빌드:** `CONFIG_LOG_DEFAULT_LEVEL_INFO=y`, `CONFIG_LOG_MAXIMUM_LEVEL_DEBUG=y`
**프로덕션 빌드:** `sdkconfig.defaults.prod`로 WARN 이상만 출력

## 3. GPS 디버깅

### 3.1 GPS 상태 확인

5초마다 자동 로그 출력 ([ublox_gps.cpp](../../main/ublox_gps.cpp)):
```
[GPS] fix=3 sat=12 rate=10.0Hz pos=(37.296096,127.206860) spd=45.2km/h hdg=182.3 alt=85.2m hdop=1.2 uart=15200B ok=150 err=0(100%)
```

| 필드 | 의미 | 정상 범위 |
|------|------|----------|
| `fix` | Fix 타입 (0=없음, 2=2D, 3=3D) | 3 (3D fix) |
| `sat` | 가시 위성 수 | 8+ |
| `rate` | NAV-PVT 수신 Hz | 10.0 (설정된 레이트) |
| `hdop` | 수평 정밀도 | <2.0 (양호), <5.0 (보통) |
| `err` | 체크섬 오류 수 | 0 |

### 3.2 GPS 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| `fix=0 sat=0` | 안테나 미연결 또는 실내 | 실외 이동, 안테나 확인 |
| `rate=0.0Hz` | UART 연결 불량 | TX/RX 배선 확인 (GPS TX → ESP32 GPIO44) |
| `rate=1.0Hz` | CFG-RATE 미적용 | 로그에서 ACK 확인, 모듈 재부팅 |
| `err` 높음 | 전기적 노이즈 | 배선 길이 단축, GND 확인 |
| `hdop` >10 | 위성 수 부족 | 하늘 시야 확보, 대기 |

### 3.3 Baud Rate 시퀀스

부팅 시 로그로 baud rate 전환 과정 확인:
```
[UBLOX_GPS] Checking if module already at 115200...
[UBLOX_GPS] Starting from 9600 baud (cold boot)...
[UBLOX_GPS] Setting baud rate to 115200...
[UBLOX_GPS] Setting 10Hz rate + Automotive mode (attempt 1)...
[UBLOX_GPS] Configuration ACK received
[UBLOX_GPS] GPS module configured: 115200 baud, 10Hz, Automotive
```

## 4. 디스플레이 디버깅

### 4.1 콜드부트 문제

LCD가 부팅 시 표시 안 됨:
1. RST 핀(GPIO21) 시퀀스 확인 — 부팅 즉시 LOW, 120ms 후 HIGH
2. Init commands — `0x11`(Sleep Out, 100ms) + `0x29`(Display On, 100ms)만 사용
3. SW Reset(`0x01`) **절대 사용 금지** — QSPI 모드에서 LCD 손상

### 4.2 LVGL 관련

- **LVGL Task:** Core 0, Priority 2, 8KB 스택
- **LVGL Tick:** 5ms
- **플러시 타임아웃:** 50ms/청크 (10 DMA 청크)
- **lvgl_mutex:** `lvgl_lock(timeout_ms)` / `lvgl_unlock()` 패턴

## 5. 빌드 문제

### 5.1 Windows Git Bash에서 idf.py 실패

MSys/Mingw 감지로 차단됨. 해결:
- ESP-IDF CMD 프롬프트 사용, 또는
- Ninja 직접 실행 (자세한 내용: [build-environment.md](../reference/build-environment.md))

### 5.2 컴파일러 경고

프로젝트 코드는 0 warnings 유지. ESP-IDF 내부 경고(FreeRTOS atomic.h)는 무시 가능.

## 6. 성능 프로파일링

```cpp
// 코드 블록 실행 시간 측정
int64_t start = esp_timer_get_time();
// ... 측정 대상 코드 ...
int64_t elapsed = esp_timer_get_time() - start;
ESP_LOGI(TAG, "Elapsed: %lld us", elapsed);
```

## 참조

- **빌드 환경:** [../reference/build-environment.md](../reference/build-environment.md)
- **하드웨어:** [../reference/hardware.md](../reference/hardware.md)
- **아키텍처:** [../reference/architecture.md](../reference/architecture.md)
