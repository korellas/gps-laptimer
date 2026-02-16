# GPS Laptimer 코드 리팩토링 분석 보고서 (최종)

**일자:** 2026-02-16
**분석 범위:** 전체 코드베이스 (~8,000줄 C/C++)
**분석 기준:** 1순위 타이밍 성능, 2순위 배터리 효율
**검토:** Claude + Codex 교차 검증 완료

---

## 종합 평가

| 항목 | 점수 | 평가 |
|------|------|------|
| **타이밍 정확도** | 7/10 | 기본 요건 충족, 일부 엣지케이스 위험 |
| **전력 효율** | 5/10 | PM 비활성화, WiFi 상시 켜짐 — 심각 |
| **코드 구조** | 8/10 | 모듈화 잘 되어 있음, 일부 대형 함수 |
| **쓰레드 안전성** | 6/10 | 대부분 "우연히" 안전, 명시적 동기화 부족 |
| **메모리 관리** | 8/10 | 정적 할당 잘 활용, PSRAM 적절히 사용 |
| **코드 완성도** | 6/10 | 미사용 모듈 다수, 미구현 함수 선언 존재 |

**종합 등급: B- (양호, 개선 필요)**

---

## 1. 타이밍/성능 이슈 (최우선)

### CRITICAL-01: LVGL 플러시 타임아웃 — 이론적 장시간 블로킹 가능

**파일:** `waveshare_display.cpp:285-304`

```cpp
const int flush_count = LCD_NATIVE_V / LCD_DMA_LINES;  // 640/64 = 10 청크
for (int i = 0; i < flush_count && err == ESP_OK; i++) {
    if (xSemaphoreTake(s_flush_done_semaphore, pdMS_TO_TICKS(200)) != pdTRUE) {
        err = ESP_ERR_TIMEOUT;  // 청크당 200ms 타임아웃
        break;                  // 첫 타임아웃 시 즉시 탈출
    }
}
```

**문제:** 각 청크의 DMA 전송이 타임아웃 직전(~199ms)에 겨우 완료되는 상황에서 10청크 x ~200ms ≈ 최대 2초 블로킹이 이론적으로 가능합니다. 이 동안 랩 크로싱 감지 불가. 첫 청크가 타임아웃하면 `break`로 즉시 탈출하므로 "단일 지연 = 즉시 2초"는 아니며, **청크별 타임아웃 누적 시 최대 2초**가 정확한 표현입니다.

**원인 참고:** LCD는 SPI DMA(GDMA), WiFi는 내부 전용 DMA를 사용하므로 DMA 채널 공유 문제는 아닙니다. 실제 블로킹 원인은 메모리 버스 대역폭 경합, 인터럽트 지연 등이 후보이나 **프로파일링으로 실측 확인 필요.**

**수정:** 타임아웃을 50ms로 줄이고, 에러 복구 로직 추가. 원인과 무관하게 방어적으로 유효합니다.

---

### CRITICAL-02: 메인 루프 시간 동기화 — 매 프레임 시스템 콜

**파일:** `main.cpp:399-410`

```cpp
// 매 프레임마다 실행 (30 Hz)
if (gApp.gpsTimeSet) {
    time_t now;
    time(&now);                    // 시스템 콜
    now += 9 * 3600;               // 수동 KST 오프셋
    struct tm kst;
    gmtime_r(&now, &kst);          // 변환
    tframe.hours = kst.tm_hour;
    tframe.minutes = kst.tm_min;
}
```

**문제:** 시간 표시는 1초에 1번만 바뀌는데, 30Hz로 매번 `time()` + `gmtime_r()` 호출. 초당 30회 불필요한 시스템 콜 + 타임존 오프셋 `9 * 3600` 하드코딩.

**수정:** 1초에 1번만 업데이트:
```cpp
static unsigned long lastTimeUpdateMs = 0;
if (now - lastTimeUpdateMs >= 1000) { ... }
```

---

### CRITICAL-03: 디스플레이 프레임 데이터 레이스 컨디션

**파일:** `main.cpp:358-367` (쓰기), `waveshare_display.cpp:877-920` (읽기)

**문제:** `lframe` (랩 프레임 데이터)이 main_task에서 mutex 없이 쓰여지고, LVGL task에서 동시에 읽힘. 32비트 쓰기는 ESP32에서 원자적이지만, 여러 필드를 함께 읽을 때 일부는 이전 값, 일부는 새 값을 볼 수 있음 (torn read).

**영향:** 디스플레이에 순간적으로 불일치 데이터 표시 가능 (옛 랩타임 + 새 델타 등).

**수정:** lframe 업데이트 시 짧은 mutex 적용.

---

### CRITICAL-04: 메인 루프 vTaskDelay(1ms)

**파일:** `main.cpp:721`

```cpp
vTaskDelay(pdMS_TO_TICKS(1));  // 1ms 최소 지연
```

**문제:** GPS 10Hz (100ms 간격) 시스템에서 1ms마다 깨어나서 폴링. 초당 1000회 중 990회는 할 일이 없음. CPU 시간 낭비 + 슬립 모드 진입 방해.

**수정:**
- **단기 (Phase 0):** `vTaskDelay(pdMS_TO_TICKS(5))` 이상으로 변경. CPU 폴링 80% 감소, 10분 작업.
- **중장기 (Phase 2):** UART 이벤트 큐 + 블로킹 수신 구조로 전환. 근본 해결이나 3시간+ 작업.

---

### CRITICAL-05: 터치 I2C 50ms 타임아웃

**파일:** `waveshare_display.cpp:381`

```cpp
esp_err_t ret = i2c_master_transmit_receive(s_touch_dev, cmd, 11, buf, 32,
                                             pdMS_TO_TICKS(50));  // 50ms!
```

**문제:** LVGL 태스크에서 5ms 틱마다 호출. I2C 지연 시 LVGL 렌더링이 50ms 블로킹 → 화면 깜빡임.

**수정:** 타임아웃 20ms로 줄이고, 타임아웃 시 마지막 유효 상태 반환.

---

### CRITICAL-06: UBX newData 판정 로직 오류 — 중복 샘플 처리

**파일:** `ublox_gps.cpp:197-199`, `ublox_gps.cpp:143`

```cpp
processUartStream(chunk[i]);
if (lastData.valid) {    // BUG: "방금 파싱 성공"이 아닌 "과거 포함 유효 상태" 체크
    newData = true;
}
```

**문제:** `lastData.valid`는 `parseUbxMessage()`에서 `fixType >= 3`일 때 true로 설정되고, fix 상실 시에만 false가 됩니다. 첫 3D fix 이후 새 메시지 파싱 없이도 UART에 잔여 바이트가 있으면 `newData = true` 반환합니다. 이는 **확인된 버그**입니다.

**실질적 영향:** `processRealGPS()`가 동일한 `lastData`를 새 데이터로 오인 → 중복 포인트 기록(`addPointToRecording()`), 중복 피니시라인 체크, 불필요한 `calculateDelta()` 호출. 중복 처리는 메시지 수신 구간에서 집중적으로 발생합니다.

**심각도:** 중간-높음. 중복 처리는 성능 낭비이며, 같은 좌표에서 피니시라인 교차는 방향 필터링과 데드존으로 대부분 방어되나, 랩 기록에 중복 GPS 포인트가 추가되는 데이터 품질 문제가 있습니다.

**수정:** `updateUBloxGPS()`에서 호출 전후 `lastData.iTOW` 비교로 "새 프레임" 판정. 또는 `processUartStream()`이 `parseUbxMessage()` 반환값을 별도 플래그로 전달.

---

### HIGH-01: 세그먼트 검색 풀 스캔 폴백

**파일:** `delta_calculator.cpp:206-225`

```cpp
if (minDistanceM >= MAX_PROJECTION_DISTANCE_M) {
    // 윈도우 검색 실패 → 전체 스캔 (O(n), n=1514)
    doSearch(0, (int)reference.points.size() - 1);
}
```

**문제:** 차량이 트랙을 벗어나거나 GPS 점프 시, 1514개 포인트 전체를 순회. 프레임 드랍 가능.

**수정:** 200개 포인트로 제한하거나 "오프트랙" 상태를 빠르게 반환.

---

### HIGH-02: exp() 신뢰도 계산 — 핫패스에서 초월함수

**파일:** `delta_calculator.cpp:285`

```cpp
result.confidence = expf(-CONFIDENCE_DECAY_FACTOR * minDistanceM / MAX_PROJECTION_DISTANCE_M);
```

**문제:** 매 델타 계산마다 `expf()` 호출 (50-100Hz). ESP32-S3의 하드웨어 FPU가 단정밀도 연산을 지원하지만 `expf()`는 여전히 소프트웨어 구현 (20-50 사이클).

**수정:** 구간 선형 근사 또는 3차 함수 근사. 단, 10-50Hz 호출률에서 초당 ~2500 사이클이므로 실질적 병목이 아닐 가능성 높음. **적용 전 `esp_timer_get_time()` 전후 측정으로 실제 비용 확인 권장.** Phase 3 (최적화)에 배치.

---

### HIGH-03: esp_timer_get_time() 디스플레이 루프 내 반복 호출

**파일:** `sector_timing.cpp:335-339`

```cpp
unsigned long elapsed = (unsigned long)(esp_timer_get_time() / 1000ULL) -
                        s_current.lastSectorCompletedAt;
if (elapsed < 5000) {
    return s_current.lastSectorDelta;
}
```

**문제:** `getDisplayDelta()`가 LVGL 렌더링에서 30-60Hz로 호출되면, 초당 1800+회 하드웨어 타이머 읽기. 섹터 완료는 랩당 3회뿐.

**수정:** 섹터 완료 시 만료 시각을 미리 계산해두고 비교.

---

### HIGH-04: sprintf 매 프레임 무조건 실행

**파일:** `waveshare_display.cpp:934-995`

```cpp
// 매 프레임 (30Hz) 실행
snprintf(timeBuf, sizeof(timeBuf), "%lu:%02lu.%lu", mins, secs, ms);
// 이후 캐시와 비교
if (strcmp(laptimeText, cache.lapTime) != 0) { ... }
```

**문제:** 캐시 비교 전에 sprintf를 항상 먼저 실행. 초당 30회 x 여러 텍스트 필드 = 초당 수백회 포맷팅.

**수정:** 원본 데이터 값이 변경됐는지 먼저 체크 후, 변경 시에만 sprintf.

---

### HIGH-05: GNSS 시간(iTOW) 파싱 후 실 랩타임 계산 미활용

**파일:** `ublox_gps.cpp:65`, `gps_processor.cpp:100`, `gps_processor.cpp:110`

```cpp
// ublox_gps.cpp
lastData.iTOW = ...

// gps_processor.cpp
point.gpsTimeMs = 0;
unsigned long lapTimeMs = now - gpsState.lapStartMs;
```

**문제:** GNSS 기준 시간은 파싱하지만 실제 랩타임은 수신 시각(`esp_timer`) 중심으로 계산합니다. UART 지연/스케줄링 지터가 타이밍에 섞일 수 있습니다.

**수정:** 랩타이밍 계산의 기준축을 `iTOW`(또는 GNSS 시각)로 일원화하고, 로컬 타이머는 보조 진단값으로 사용.

---

### MEDIUM-01: cos(lat) 단일 캐싱으로 거리 오차 (영향도 낮음)

**파일:** `delta_calculator.cpp:181-184`

```cpp
float cosLat = s_cosLat;
if (cosLat == 0.0f) {
    cosLat = (float)cos(current.lat * GEO_DEG_TO_RAD);
}
```

**현상:** `s_cosLat`은 레퍼런스 랩 첫 포인트의 위도로 한 번만 계산. 차량이 남북으로 이동하면 cos(lat)이 달라져야 하나 반영 안 됨.

**영향도 분석:** 델타 계산에서 중요한 것은 절대 거리가 아니라 현재 랩과 레퍼런스 랩의 **상대 거리**입니다. 양쪽 모두 동일한 `s_cosLat`으로 계산되므로 cos(lat) 오차가 **체계적으로 상쇄**됩니다. 에버랜드 트랙(남북 ~0.01°)에서 cos(37.29°) vs cos(37.30°)의 차이는 0.014%이며, 델타에 대한 실질적 오차는 무시 가능합니다. **5km 이상의 대형 트랙(남북 0.05°+)에서만 의미 있는 이슈.**

---

### MEDIUM-02: GPS 필터링에서 sin/cos/atan2 매번 계산

**파일:** `gps_filter.cpp:150-163`

```cpp
sumSin += sin(headingRad) * weight;
sumCos += cos(headingRad) * weight;
smoothed.headingDeg = atan2(sumSin, sumCos) * GEO_RAD_TO_DEG;
```

**문제:** 방향 평균에 초월함수 3개 사용 (10Hz GPS마다). 알고리즘은 정확하나 비용이 큼.

---

### MEDIUM-03: float 정밀도 손실 — 좌표 변환

**파일:** `geo_utils.cpp:32`

```cpp
float avgLatRad = (float)((lat1 + lat2) * 0.5 * GEO_DEG_TO_RAD);
```

**문제:** double 연산 결과를 float으로 캐스팅. 에버랜드(37.297°N)에서 ~0.11m 오차. 수백 번 누적 시 수 미터 오차.

---

## 2. 전력/배터리 이슈 (2순위)

### POWER-01: 전원 관리 완전 비활성화 (CRITICAL)

**파일:** `sdkconfig`

```
# CONFIG_PM_ENABLE is not set
```

**현재 상태:**
- CPU 항상 160MHz 고정 (`sdkconfig:2831`)
- 동적 주파수 스케일링(DFS) 불가
- 라이트 슬립 불가
- WiFi 모뎀 슬립 불가

**영향:**
- 유휴 시 CPU 25-40mA 소모 (PM 활성화 시 5-8mA 가능)
- GPS 대기 중 (100ms 중 97ms 유휴) CPU가 160MHz로 계속 동작

**수정:** `CONFIG_PM_ENABLE=y` + `CONFIG_FREERTOS_USE_TICKLESS_IDLE=y`

**예상 절약:** 20-35mW (유휴 시 50-60% 감소)

---

### POWER-02: WiFi SoftAP 항상 켜짐 (CRITICAL)

**파일:** `main.cpp:595`

```cpp
initWifiPortal();  // 부팅 시 무조건 시작, 끄는 코드 없음
```

**현재 소모:**
```
WiFi SoftAP 라디오:    50-80mA (비콘 전송)
WiFi MAC 처리:         10-20mA
WiFi CPU 관리:         5-10mA
합계:                  80-120mA (전체 전력의 40-60%!)
```

**실제 사용 패턴:** WiFi는 설정 변경 시에만 필요. 주행 중에는 불필요.

**수정:** "레이스 모드 기본 OFF + 필요 시 명시적 ON" 정책 적용. 구체적으로: 시작 화면에서 WiFi ON/OFF 토글 추가 (기본 OFF), 시리얼 명령 `w`로 토글, 5분 무접속 시 자동 OFF.

**예상 절약:** 80-120mW (WiFi 끄면 배터리 수명 2-3배 증가)

---

### POWER-03: FreeRTOS Tickless Idle 미설정

**파일:** `sdkconfig`

```
CONFIG_FREERTOS_HZ=1000  // 1ms 시스템 틱
# CONFIG_FREERTOS_USE_TICKLESS_IDLE is not set
```

**문제:** 모든 태스크가 블록되어도 1ms마다 타이머 인터럽트로 깨어남. Tickless Idle 사용 시 다음 태스크 시각까지 슬립 가능.

---

### POWER-04: GPS UART 폴링 방식

**파일:** `ublox_gps.cpp:188-210`

```cpp
// 0 타임아웃 논블로킹 폴링
while ((len = uart_read_bytes(kUartNum, chunk, sizeof(chunk), 0)) > 0) { ... }
```

**문제:**
- UART 이벤트 큐/인터럽트 미사용 (`uart_driver_install` 시 큐 NULL)
- 메인 루프에서 1ms마다 UART 버퍼 체크 → 초당 1000회 중 990회는 빈 체크
- ESP32-S3 UART는 RX 인터럽트, 패턴 매칭 등 하드웨어 기능 지원

**수정:** `uart_driver_install` 시 이벤트 큐 연결 + 전용 태스크에서 `xQueueReceive` 블로킹.

**예상 절약:** 2-5mW

---

### POWER-05: 로깅 레벨 INFO (프로덕션에 부적합)

**파일:** `sdkconfig`

```
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_LOG_MAXIMUM_LEVEL_DEBUG=y
```

**문제:** USB 시리얼 출력마다 ~500us. 정상 동작 시 초당 1-3회 INFO 로그 → 2-5mA 추가 소모.

**수정:** 프로덕션 빌드에서 `CONFIG_LOG_DEFAULT_LEVEL_WARN=y`

---

### POWER-06: CPU 주파수/PM 동적 정책 부재

**파일:** `sdkconfig`

```
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_160=y    # sdkconfig:2831
CONFIG_COMPILER_OPTIMIZATION_PERF=y
```

**분석:**
| 컴포넌트 | 필요 주파수 | 현재 |
|----------|-----------|------|
| GPS UART 폴링 | 80MHz | 160MHz |
| 디스플레이 렌더링 | 160MHz | 160MHz |
| 델타 계산 | 80MHz | 160MHz |
| 터치 I2C | 80MHz | 160MHz |

**결론:** 현재 160MHz 설정은 타당합니다. 핵심 이슈는 고정 주파수가 아니라 **DFS(Dynamic Frequency Scaling) 정책 부재**입니다. DFS 활성화 시 80MHz(유휴)↔160MHz(활성) 전환이 가능하며, 이것이 실제 절약 포인트입니다.

---

### 배터리 수명 예측

```
현재 (PM 비활성, WiFi 상시):
  CPU: 25-40mA + WiFi: 80-120mA + 디스플레이: 20-30mA + GPS: 1-2mA
  합계: ~150mA → 3200mAh 기준 약 4시간

Phase 1 적용 후 (PM 활성, WiFi 타임아웃):
  CPU: 8-15mA + WiFi: 0mA + 디스플레이: 20-30mA + GPS: 1-2mA
  합계: ~40mA → 3200mAh 기준 약 13시간
```

---

## 3. 미사용/데드 코드

### DEAD-01: sector.cpp/h — 초기화만 호출, 런타임 기능 미사용 (495줄)

**파일:** `components/track/sector.cpp` (254줄), `sector.h` (241줄)

**상태:** `initSectorDetection()`만 `simulation.cpp:123`에서 호출됩니다. 이 함수는 `s_initialized = true` + `resetSectorState()`만 수행합니다. 핵심 런타임 함수인 `updateSectorDetection()`은 어디에서도 호출되지 않으며, `sector_timing.cpp`의 거리 기반 방식으로 완전 대체되었습니다. 컴파일되어 ~1.5KB 플래시 + ~500B RAM 낭비.

**조치:** 삭제. `simulation.cpp:123-124`의 `initSectorDetection()` 호출 및 extern 선언도 함께 제거 (DEAD-07 참조).

---

### DEAD-02: 미구현 함수 선언 4개 — 링크 실패 위험

**파일:** `components/modes/gps_processor.h:75-112`

```cpp
bool hasGPSFix();                    // 선언만, 구현 없음
unsigned long getRealLapStartMs();   // 선언만, 구현 없음
void setRealLapStartMs(unsigned long startMs);  // 선언만, 구현 없음
void forceStartLap();               // 선언만, 구현 없음
```

**위험:** 미래에 누군가 이 함수를 호출하면 링크 에러 발생.

**조치:** 선언 제거 또는 구현 추가.

---

### DEAD-03: finish_line.h 미구현 함수 선언 2개

**파일:** `main/finish_line.h:66-67`

```cpp
bool autoSelectFinishLine(double lat, double lng);    // 구현 없음
const TrackTemplate* getActiveTrackTemplate();         // 구현 없음, TrackTemplate 타입도 없음
```

**조치:** 선언 제거 (track_manager로 대체됨).

---

### DEAD-04: app_controller/app_state (834줄) — 완전히 미사용

이미 알려진 이슈. AppContext 패턴으로 대체됨. 아직 삭제되지 않음.

**조치:** 삭제 (로드맵 Phase 1 계획)

---

### DEAD-05: 미사용 함수/변수 (컴파일러 경고)

| 위치 | 항목 | 상태 |
|------|------|------|
| `finish_line.cpp:75` | `pointToLineDistance()` 함수 | 미사용 |
| `dead_reckoning.cpp` | `elapsedSinceUpdate` 변수 | 미사용 |
| `sector.cpp:120` | `currentSector` 변수 | 미사용 (모듈 자체가 런타임 미사용) |
| `gps_filter.cpp:133` | `sumHeading` 변수 | 미사용 |
| `waveshare_display.cpp:1155-1164` | `updateGpsData()`, `updateTimeData()` | 스텁 (빈 함수) |

---

### DEAD-06: DEBUG_OUTPUT 매크로 — 해소됨

**파일:** `config.h:20-21`, `sector_timing.cpp:153, 189, 195`

**상태:** `DEBUG_OUTPUT`은 `config.h`에 `extern bool g_debugOutput; #define DEBUG_OUTPUT g_debugOutput`로 정의되어 있으며, 런타임 토글 가능합니다. **미정의 이슈는 해소 상태.**

**후속 점검:** `DEBUG_OUTPUT` 사용처에서 `printf()` 대신 `ESP_LOGD()` 통일 여부 검토 → 로그 레벨 정책 일관성 확보.

---

### DEAD-07: simulation.cpp에서 죽은 모듈 호출

**파일:** `simulation.cpp:123-124`

```cpp
extern void initSectorDetection();
initSectorDetection();   // ← sector.cpp (런타임 미사용 모듈) 호출
initSectorTiming();      // ← 실제 사용되는 모듈
```

**조치:** `initSectorDetection()` 호출 및 extern 선언 제거.

---

## 4. 하드코딩 값들

### 중요도 높음

| 파일 | 라인 | 값 | 문제 |
|------|------|-----|------|
| `sector_timing.cpp:402-405` | 에버랜드 섹터 좌표 | 제네릭 함수 안에 특정 트랙 좌표 하드코딩 | 다중 트랙 불가 |
| `main.cpp:401` | `9 * 3600` (KST) | 타임존 하드코딩 | 해외 트랙 사용 시 시간 틀림 |
| `sector_timing.cpp:337` | `5000` | 섹터 델타 표시 시간 매직 넘버 | config.h에 있어야 함 |
| `sector_timing.cpp:303` | `2.0f` | 진행률 클램프 최대값 | 느린 랩에서 잘림 |
| `simulation.cpp:118` | `"everland"` | 시뮬레이션 트랙 ID | 다중 트랙 불가 |

### 중요도 중간

| 파일 | 라인 | 값 | 문제 |
|------|------|-----|------|
| `waveshare_display.cpp:213-216` | `0.15f`, `0.20f` | EMA 알파 상수 | 튜닝 시 코드 수정 필요 |
| `waveshare_display.cpp:1245` | `30` 픽셀 | 스와이프 임계값 | config.h에 있어야 함 |
| `delta_calculator.cpp:260` | `0.02f` 초 | 세그먼트 시간 임계값 | GPS 주파수 의존적 |
| `waveshare_display.cpp:379` | `0xb5, 0xab...` | 터치 프로토콜 매직 바이트 | 상수 정의 필요 |
| `finish_line.h:18-26` | 8개 상수 | 피니시라인 설정 | `config.h`에 있어야 함 |

### 좌표 데이터 3중 중복 (DRY 위반)

에버랜드 섹터 경계 좌표가 3곳에 동일하게 정의됨:
1. `builtin_tracks.h:31-35` — constexpr 개별 변수
2. `builtin_tracks.h:43-46` — SECTOR_BOUNDARIES 배열
3. `sector_timing.cpp:402-405` — 로컬 배열

좌표 수정 시 3곳 모두 변경해야 함. 실수 위험.

---

## 5. 알고리즘/설계 비효율

### ALGO-01: O(N) 섹터 경계 검색 — 잘못된 거리 메트릭 사용

**파일:** `sector_timing.cpp:414-422`

```cpp
float dx = (float)(refPoints[i].lat - boundaries[b].lat);
float dy = (float)(refPoints[i].lng - boundaries[b].lng);
float distSq = dx * dx + dy * dy;  // 도(degree) 단위 제곱거리!
```

**문제:**
1. 위도/경도 도 단위 제곱거리를 미터 거리 대신 사용 — 위도 37°에서 위도 0.001° ≈ 111m, 경도 0.001° ≈ 88m. 스케일이 다름.
2. `minDist` 초기값 `1e9f` — 단위와 무관한 임의의 센티넬 값.
3. **결과적으로는 작동하나**, 의미적으로 틀림. 경계가 수십 미터 틀릴 수 있음.

**수정:** `fastDistanceMeters()` 사용, 초기값 `FLT_MAX`으로 변경.

---

### ALGO-02: 섹터 경계 재계산 — 정당하나 거리 메트릭 개선 필요

**파일:** `sector_timing.cpp:391-440`

**분석:** `updateSectorDistancesFromReference()`는 레퍼런스 랩의 `cumDist[]`에서 섹터 경계 좌표에 가장 가까운 포인트의 누적 거리를 찾습니다. 다른 레퍼런스 랩(다른 주행 라인)은 같은 GPS 좌표에서 다른 누적 거리를 가지므로, **레퍼런스 변경 시 재계산은 정당합니다.**

**개선점:** 1514포인트 x 2경계 x 4연산 = ~12,000 float 연산으로 랩 완료 처리 중 6ms 블로킹. 비핫패스(랩당 1회)이므로 성능 영향은 낮으나, ALGO-01에서 지적한 degree-space 거리 메트릭을 `fastDistanceMeters()`로 개선할 필요가 있습니다.

---

### ALGO-03: 람다 클로저 오버헤드 — 핫패스 세그먼트 검색

**파일:** `delta_calculator.cpp:186-204`

```cpp
auto doSearch = [&](int start, int end) {
    for (int i = start; i < end; i++) { ... }
};
```

**문제:** 캡처-바이-레퍼런스 람다가 핫패스에서 매번 생성. 정적 함수로 추출하면 클로저 셋업 오버헤드 제거.

---

### ALGO-04: GPS 스파이크 감지 — 빠른 체크 없이 바로 거리 계산

**파일:** `gps_filter.cpp:65`

```cpp
result.jumpDistanceM = fastDistanceMeters(lastValid.lat, lastValid.lng,
                                           point.lat, point.lng);
```

**수정:** lat/lng 차이 빠른 체크 먼저:
```cpp
if (fabs(point.lat - lastValid.lat) > 0.05 ||
    fabs(point.lng - lastValid.lng) > 0.05) {
    return {true, 1000.0f, 0.0f};  // 명백한 스파이크
}
```

---

### ALGO-05: SEGMENT_SEARCH_WINDOW 고정 크기

**파일:** `config.h`

```cpp
constexpr int SEGMENT_SEARCH_WINDOW = 50;
```

**문제:** 에버랜드(1514포인트)에 맞춰진 고정값. 5km 트랙 (5000포인트)에서는 윈도우가 너무 작아 풀스캔 폴백 빈번.

**수정:** 레퍼런스 랩 크기의 ~3%로 동적 설정.

---

## 6. 하드웨어 활용 부족

### HW-01: ESP32-S3 하드웨어 타이머 4개 미사용

```
sdkconfig: CONFIG_SOC_TIMER_GROUP_TOTAL_TIMERS=4
현재 사용: 0개 (esp_timer로 LVGL 틱만 사용)
```

GPS 폴링, 디스플레이 업데이트 등을 소프트웨어 루프 대신 하드웨어 타이머로 전환 가능.

---

### HW-02: ULP 코프로세서 미사용

```
sdkconfig: CONFIG_SOC_ULP_SUPPORTED=y
현재 사용: 없음
```

딥 슬립 중 ULP로 전원 버튼 감시 + 배터리 ADC 모니터링 가능. 소비 전류 0.001mA.

현재는 메인 루프에서 1ms마다 GPIO16 폴링:
```cpp
static void checkPowerButton(void) {
    bool pressed = (gpio_get_level((gpio_num_t)PWR_BUTTON_PIN) == 0);
    // ...
}
```

---

### HW-03: UART RX 인터럽트/패턴 매칭 미사용

```
sdkconfig: CONFIG_SOC_UART_SUPPORT_WAKEUP_INT=y  // 지원됨
현재: 폴링 방식
```

UBX 싱크 바이트 (0xB5, 0x62) 패턴 매칭으로 자동 감지 가능.

---

### HW-04: 디스플레이 적응적 리프레시 미사용

현재 60Hz 고정 업데이트 (`DISPLAY_UPDATE_INTERVAL_MS = 16`). 데이터 변경 없을 때도 렌더링.

GPS fix 대기 중이나 유휴 시 10-30Hz로 감소 가능.

---

### HW-05: u-blox 출력률/메시지 구성(CFG) 송신 코드 부재 + Baud Rate 제약 *(known/deferred)*

**파일:** `main/ublox_gps.cpp`, `main/ublox_gps.h`

> **참고:** GPS 하드웨어 모드는 아직 미완성 known 영역입니다. 아래 내용은 GPS 모드 본격 구현 시 함께 처리할 항목으로 기록합니다.

**문제 1 — CFG 부재:** UART 수신/파싱은 구현되어 있으나, 모듈 출력률/메시지 설정(CFG-RATE 등) 송신 코드가 없습니다. u-blox MAX-M10S 기본 출력률은 **1Hz** (공장 설정). 문서/코드 주석은 "10Hz GPS" 가정이나 10Hz를 설정하는 코드가 없음. 모듈이 이전 세션에서 10Hz로 설정되어 NVM에 저장된 경우 유지되나, 새 모듈이나 공장 초기화 후에는 1Hz로 복귀합니다.

**문제 2 — Baud Rate 물리적 제약:** `UBLOX_BAUD = 9600` (8N1 = 최대 960 bytes/sec). NAV-PVT 메시지 크기: 6(헤더) + 92(페이로드) + 2(체크섬) = **100 bytes**. 10Hz = 100 x 10 = **1000 bytes/sec > 960 bytes/sec 한계.** 현재 9600 baud에서는 10Hz NAV-PVT가 물리적으로 전송 불가합니다. 최대 ~9Hz이며, 다른 UBX 메시지까지 합치면 실질 6-7Hz.

**조치 (GPS 모드 구현 시):**
1. `UBLOX_BAUD`를 **38400 또는 115200으로 상향** + UBX-CFG-PRT로 모듈 baud rate 변경
2. 부팅 시 UBX-CFG-RATE (100ms=10Hz) + UBX-CFG-MSG (NAV-PVT만 활성화) 전송
3. ACK 확인 포함

---

## 7. 코드 품질/구조

### QUALITY-01: 태스크 우선순위 — 현재 구조 적절, 실질적 이슈는 mutex 경합

```
현재:
  Priority 5: Main Task (GPS 처리) — Core 1
  Priority 2: LVGL Task (디스플레이) — Core 0
  Priority 1: Battery Task + Timer Service
```

**분석:** Main Task(Core 1)와 LVGL Task(Core 0)는 **서로 다른 코어에 핀닝**되어 있으므로, ESP32-S3 SMP FreeRTOS에서 **직접 선점 관계는 없습니다.** 각 코어는 독립적으로 자신의 최고 우선순위 태스크를 실행합니다. 따라서 우선순위 수치만으로 선점을 논하기 어렵고, 현재 구조(Main 5, LVGL 2) 자체는 문제가 아닙니다.

**실질적 이슈:** 다만 `lvgl_mutex`/공유자원 경합을 통한 **간접적 영향**은 존재합니다. Main이 `lvgl_lock()`을 잡고 lframe을 쓸 때 LVGL이 대기, 또는 LVGL이 렌더링 중일 때 Main이 대기합니다. 이 경합에서 mutex 해제 후 어느 태스크가 먼저 획득하는지에 우선순위가 영향을 줄 수 있습니다. 1ms 폴링(CRITICAL-04)이 불필요한 mutex 획득 시도를 유발하며, **이벤트 기반 전환(POWER-04)이 근본 해결.**

---

### QUALITY-02: updateDisplayData() — 84줄 함수

**파일:** `main.cpp:343-426`

GPS 프레임 + 랩 프레임 + 시간 프레임 + 시스템 시간 동기화를 한 함수에서 처리. 4개 함수로 분리 필요.

---

### QUALITY-03: 메모리 할당 실패 시 조용한 반환

**파일:** `waveshare_display.cpp:662-669`

```cpp
if (!buf1 || !buf2 || !s_lvgl_rot_buf || !s_lvgl_dma_buf) {
    ESP_LOGE(TAG, "Failed to allocate LVGL buffers");
    return;  // ← 조용히 반환, 이후 NULL 참조 크래시
}
```

**수정:** `ESP_ERROR_CHECK(ESP_ERR_NO_MEM)` 으로 패닉 트리거.

---

### QUALITY-04: simulation.cpp에서 함수 내부 extern 선언

**파일:** `simulation.cpp:117-118, 123-124`

```cpp
void initializeSimulation() {
    extern bool setActiveTrackById(const char* trackId, const char* layoutId);
    extern void initSectorDetection();
    // ...
}
```

**문제:** 비표준 패턴. 헤더 파일 #include로 대체해야 함.

---

### QUALITY-05: SimulationState::reset() — 해소됨

**파일:** `simulation.h:37-43`

최신 코드에서 `reset()`은 `initialized = false`로 정상 리셋합니다 (`simulation.h:41` 확인). **이 항목은 이슈 없음.**

---

### QUALITY-06: `GPSPoint::isValid()` 계약과 실제 사용 패턴 불일치

**파일:** `components/common/types.h:47-55`, `gps_processor.cpp:132`, `simulation.cpp:280`

**문제:** `isValid()`는 `initialized` 플래그 기반인데, 주 경로는 `.set()` 대신 구조체 필드 직접 대입(`gApp.currentPoint = point`) 중심이라 `initialized`가 false로 유지됩니다.

**검증:**
- `gps_processor.cpp:95-101`: GPSPoint를 필드별 직접 대입, `.set()` 미호출 → `initialized = false`
- `simulation.cpp:97-103` (`loadSimulationLap`): 동일 패턴
- 결과적으로 모든 런타임 GPS 포인트에서 `isValid()`가 false 반환

**영향:** 현재 코드에서 `isValid()`를 호출하는 곳이 거의 없어 즉각적 버그는 아니지만, 향후 `isValid()` 기반 로직 추가 시 전면 실패 위험.

**수정:** `initialized` 플래그 대신 `lat != 0.0 || lng != 0.0` 기반으로 재정의하거나, 모든 GPSPoint 생성 경로에 `initialized = true` 추가.

---

### QUALITY-07: 실GPS 첫 베스트랩 이후 referenceLap 메모리 동기화 누락 (HIGH)

**파일:** `main.cpp:474`, `main.cpp:491`

**문제:** 첫 베스트랩 완료 시:
- `saveBestLap(completedLap)` → SPIFFS 저장만 수행 (`lap_storage.cpp:245-274`)
- `hasValidReferenceLap = true` 설정, 그러나 `gApp.referenceLap` 갱신 없음

**시뮬레이션 모드:** `onSimLapComplete()`에서 `loadRefLapFromTrackData()` 호출로 갱신되어 문제 없음.

**GPS 모드 (선행 레퍼런스 없이):** 첫 랩 완료 → `hasValidReferenceLap = true` → `gApp.referenceLap` 비어있음 → 다음 랩에서 `calculateDelta()`가 빈 레퍼런스로 호출. `delta_calculator.cpp:168`에서 `reference.points.size() < 2` 체크로 즉시 반환하므로 **크래시는 아니나**, 화면에 **무의미한 델타(0.00 고정)**가 계속 표시되는 UX 문제 발생.

**수정:** `onLapComplete()`에서 새 베스트랩을 즉시 `setReferenceLap()`으로 메모리에 반영.

---

### QUALITY-08: `gps_filter`/`dead_reckoning` 기능이 실GPS 경로에 미연결

**파일:** `components/geo/gps_filter.cpp`, `components/geo/dead_reckoning.cpp`, `components/modes/gps_processor.cpp`

**문제:** 필터/DR 모듈은 구현되어 있으나, `processRealGPS()` 파이프라인에서 `smoothPoint`, `filterGPSPoint`, `deadReckon`, `updateDeadReckoning` 등이 전혀 호출되지 않습니다. raw GPS 데이터가 직접 `calculateDelta()`로 전달됩니다.

**위험:** 시뮬레이션 데이터는 깨끗하므로 문제 없으나, 실 GPS에서는 노이즈/스파이크가 필터 없이 델타 계산에 영향 → 화면 깜빡임, 잘못된 섹터 전환 가능.

**주의:** 필터 연결 시 지연(latency) 도입 가능. 실 트랙 테스트 전 on/off 비교 필수.

**조치:** `processRealGPS()` 파이프라인에 필터/DR 단계를 명시적으로 연결하고, on/off 정책을 설정값으로 제어.

---

## 8. 수정 우선순위 로드맵

### Phase 0: 즉시 (2-3시간)

| # | 이슈 | 파일 | 노력 |
|---|------|------|------|
| 1 | PM 활성화 (`CONFIG_PM_ENABLE=y`) | sdkconfig | 30분 |
| 2 | Tickless Idle 활성화 | sdkconfig | 30분 |
| 3 | WiFi 유휴 타임아웃 (5분, 기본 OFF) | main.cpp | 2시간 |
| 4 | vTaskDelay(1) → vTaskDelay(5) | main.cpp:721 | 10분 |
| 5 | 터치 I2C 타임아웃 50→20ms | waveshare_display.cpp:381 | 10분 |
| 5e | GPS 첫 베스트랩 → referenceLap 즉시 반영 | main.cpp | 30분 |

**예상 효과:** 배터리 수명 2-3배 향상

### Phase 0-GPS: GPS 모드 본격 구현 시 *(known/deferred)*

| # | 이슈 | 파일 | 노력 |
|---|------|------|------|
| 5b | UART baud rate 9600→115200 + 모듈 CFG-PRT | ublox_gps.cpp/h | 30분 |
| 5c | GPS CFG-RATE 10Hz 설정 송신 + ACK 확인 | ublox_gps.cpp | 1시간 |
| 5d | UBX newData 판정 로직 수정 (iTOW 비교) | ublox_gps.cpp | 30분 |

**참고:** GPS 하드웨어 모드 미완성 known 영역. 시뮬레이션 모드에는 영향 없음.

### Phase 1: 다음 스프린트 (4-6시간)

| # | 이슈 | 파일 | 노력 |
|---|------|------|------|
| 6 | 데드 모듈 삭제 (sector.cpp/h, app_*) | 여러 파일 | 1시간 |
| 7 | 미구현 함수 선언 6개 제거 | gps_processor.h, finish_line.h | 30분 |
| 8 | lframe 레이스 컨디션 수정 (mutex) | waveshare_display.cpp, main.cpp | 30분 |
| 9 | 시간 업데이트 1초/회로 제한 | main.cpp:399-410 | 15분 |
| 10 | LVGL 플러시 타임아웃 200→50ms | waveshare_display.cpp:295 | 30분 |
| 11 | DEBUG_OUTPUT printf→ESP_LOGD 통일 검토 | config.h, sector_timing.cpp | 10분 |
| 12 | 매직넘버 config.h 이동 (5000, 2.0f 등) | sector_timing.cpp, config.h | 30분 |
| 13 | 좌표 3중 중복 통합 | builtin_tracks.h, sector_timing.cpp | 1시간 |

### Phase 2: 아키텍처 (6-10시간)

| # | 이슈 | 파일 | 노력 |
|---|------|------|------|
| 14 | TrackLayout에 섹터 경계 좌표 통합 | track_types.h, sector_timing.cpp | 3시간 |
| 15 | GPS UART 이벤트 큐 방식 전환 | ublox_gps.cpp | 3시간 |
| 16 | lvgl_mutex 경합 최적화 | main.cpp, waveshare_display.cpp | 2시간 |
| 17 | 프로덕션용 sdkconfig.prod 생성 | sdkconfig.prod | 1시간 |

### Phase 3: 최적화 (4-6시간)

| # | 이슈 | 파일 | 노력 |
|---|------|------|------|
| 18 | exp() → 근사 함수 (프로파일링 후 결정) | delta_calculator.cpp | 30분 |
| 19 | sprintf 조건부 실행 | waveshare_display.cpp | 1시간 |
| 20 | 세그먼트 검색 최적화 (풀스캔 제한) | delta_calculator.cpp | 1시간 |
| 21 | 적응적 디스플레이 리프레시 | main.cpp, config.h | 2시간 |
| 22 | 컴파일러 경고 전부 수정 | 여러 파일 | 1시간 |

---

## 9. 문서 관련 이슈

### DOC-01: hardware.md 디스플레이 GPIO 핀번호 완전 오류
docs에 기재된 QSPI 핀 (39/47/18/7/48/5/17)과 실제 코드 핀 (10/11/12/13/14/9/21/8)이 전혀 다름.

### DOC-02: architecture.md / ui-specification.md — LVGL 틱 16ms → 5ms 미반영

### DOC-03: data-structures.md — AppContext 필드명/타입 불일치
`mode` → `currentGpsMode`, `currentGPS` → `currentPoint` 등. 9개 새 필드 누락.

### DOC-04: roadmap.md — Phase 2.4 (시작화면), 2.5 (배터리) "계획됨" → 이미 구현 완료

### DOC-05: changelog.md — WiFi, 배터리 EWMA, ADC 오버샘플링, GPIO16 전원오프, 시작화면 미기록

### DOC-06: status.md — 2026-02-08 이후 구현 기능 전혀 반영 안 됨

### DOC-07: debugging.md — 여러 곳에서 참조하지만 파일 자체가 존재하지 않음

---

## 10. 리팩토링 진행 현황

**작업 브랜치:** `refactor/code-review-cleanup` (from `feature/esp-idf-migration`)
**작업 일자:** 2026-02-16

### Phase 0: 즉시 수정 — 완료

| # | 이슈 | 상태 | 비고 |
|---|------|------|------|
| 0-1 | PM_ENABLE 활성화 | **완료** | sdkconfig: DFS 80-160MHz |
| 0-2 | Tickless Idle 활성화 | **완료** | sdkconfig: CONFIG_FREERTOS_USE_TICKLESS_IDLE=y |
| 0-3 | WiFi 라이프사이클 관리 | **완료** | 시작 화면에서 ON, 클라이언트 없으면 레이스 진입 시 OFF, 클라이언트 있으면 유지. `w` 시리얼 명령으로 수동 토글 가능 |
| 0-4 | vTaskDelay(1) → vTaskDelay(5) | **완료** | main.cpp main loop |
| 0-5 | 터치 I2C 타임아웃 50→20ms | **완료** | waveshare_display.cpp |
| 0-5e | GPS 모드 베스트 랩 → referenceLap 즉시 반영 | **완료** | `convertStorableToLapData()` 헬퍼 추출, `onLapComplete()`에서 즉시 `setReferenceLap()` |

### Phase 1: 다음 스프린트 — 완료

| # | 이슈 | 상태 | 비고 |
|---|------|------|------|
| 6 | 데드 모듈 삭제 | **완료** | `sector.cpp/h` 삭제, `app_controller/app_state`는 이전 작업에서 이미 삭제됨 |
| 7 | 미구현 함수 선언 제거 | **완료** | `gps_processor.h` 6개, `finish_line.h` 2개 제거. `initializeGPSProcessor()` → static으로 변경 |
| 8 | lframe 레이스 컨디션 | **해당없음** | 분석 결과 실제 race condition 아님. frame 읽기/쓰기 모두 main_task(Core 1)에서 실행. LVGL 위젯 접근은 기존 `lvgl_mutex`로 보호됨 |
| 9 | 시간 업데이트 1초/회 제한 | **완료** | `lastTimeUpdateMs` 추가, 30Hz→1Hz |
| 10 | LVGL 플러시 타임아웃 200→50ms | **완료** | 3개소 일괄 변경 |
| 11 | DEBUG_OUTPUT → ESP_LOGD 통일 | **완료** | `sector_timing.cpp`, `dead_reckoning.cpp`, `gps_filter.cpp`, `simulation.cpp` — ESP_LOGD + TAG 패턴 적용 |
| 12 | 매직넘버 config.h 이동 | **완료** | `5000` → `SECTOR_DELTA_DISPLAY_MS`, `2.0f` → `SECTOR_PROGRESS_MAX` |
| 13 | 좌표 중복 통합 | **완료** | `sector_timing.cpp` 하드코딩 제거 → `SectorBoundaryPoint` 파라미터로 전달. SSOT: `builtin_tracks.h` |

### 빌드 이슈 및 해결

| 이슈 | 원인 | 해결 |
|------|------|------|
| `esp_timer.h: No such file or directory` (wifi_portal.cpp) | CMakeLists.txt에 esp_timer 의존성 누락 | `components/wifi_portal/CMakeLists.txt` REQUIRES에 `esp_timer` 추가 |
| `convertStorableToLapData was not declared` (main.cpp) | 함수 정의(518줄)가 호출(480줄)보다 뒤에 위치 | `static` forward declaration 추가 |
| `initializeGPSProcessor was not declared` (gps_processor.cpp) | 헤더에서 선언 제거 후 cpp에서도 접근 불가 | `initializeGPSProcessor()` → `static`, `initializeGPSMode()` public wrapper 유지 |

### 변경된 파일 목록

- `sdkconfig` — PM, Tickless Idle
- `main/main.cpp` — PM 설정, WiFi 라이프사이클, vTaskDelay, referenceLap 즉시반영, 시간 업데이트 throttle
- `main/waveshare_display.cpp` — 터치 타임아웃, LVGL 플러시 타임아웃
- `main/waveshare_display.h` — (변경 없음)
- `main/serial_commands.cpp` — WiFi 토글 `w` 명령
- `main/finish_line.h` — 미구현 선언 제거
- `components/wifi_portal/wifi_portal.h` — start/stop/toggle/isActive API
- `components/wifi_portal/wifi_portal.cpp` — init/start/stop 라이프사이클 분리
- `components/wifi_portal/CMakeLists.txt` — esp_timer 의존성
- `components/modes/gps_processor.h` — 미구현 선언 6개 제거
- `components/modes/gps_processor.cpp` — initializeGPSProcessor static화
- `components/modes/simulation.cpp` — initSectorDetection 제거, ESP_LOGD 전환, 섹터 좌표 파라미터화
- `components/common/config.h` — `SECTOR_DELTA_DISPLAY_MS`, `SECTOR_PROGRESS_MAX` 추가
- `components/track/sector.cpp` — **삭제**
- `components/track/sector.h` — **삭제**
- `components/track/CMakeLists.txt` — sector.cpp 제거
- `components/timing/sector_timing.h` — `SectorBoundaryPoint` struct, `updateSectorDistancesFromReference` 시그니처 변경
- `components/timing/sector_timing.cpp` — ESP_LOGD 전환, 하드코딩 좌표 제거, 매직넘버→상수
- `components/geo/dead_reckoning.cpp` — ESP_LOGD 전환
- `components/geo/gps_filter.cpp` — ESP_LOGD 전환
- `components/track/track_types.h` — FinishLine→FinishLineDefinition (replace_all)
- `components/track/track_manager.h` — FinishLineDefinition 반환타입 변경
- `components/track/track_manager.cpp` — FinishLineDefinition (replace_all)
- `main/waveshare_display.cpp` — updateLapData() 연산/LVGL 분리, startup TAP mutex 통합, lvgl_lock/unlock 사용
- `components/wifi_portal/wifi_portal.cpp` — httpd_uri 필드 초기화 추가 (경고 수정)
- `components/dns_server/include/dns_server.h` — DNS_SERVER_CONFIG_SINGLE ip 초기화 추가
- `sdkconfig.defaults.prod` — **신규** (프로덕션 빌드 오버레이)

### 남은 작업 — Phase 2/3

**Phase 0-GPS, #15 (GPS UART 이벤트 큐)는 GPS 코드 작업 완료 후 별도 검토 예정.**

| Phase | # | 이슈 | 상태 | 비고 |
|-------|---|------|------|------|
| 2 | 14 | TrackLayout 섹터 경계 좌표 통합 | **완료** | FinishLine→FinishLineDefinition 리네임, extern 제거 |
| 2 | 15 | GPS UART 이벤트 큐 방식 전환 | **GPS 작업 후** | GPS 코드 수정 중이므로 보류 |
| 2 | 16 | lvgl_mutex 경합 최적화 | **완료** | updateLapData() 연산/LVGL 분리, startup TAP 통합 |
| 2 | 17 | 프로덕션용 sdkconfig.prod | **완료** | `sdkconfig.defaults.prod` 생성 (WARN 로그, assertion silent) |
| 3 | 18 | exp() → 근사 함수 | **보류** | 10-50Hz 호출, ~2500 cycles/sec — 프로파일링 후 결정 |
| 3 | 19 | sprintf 조건부 실행 | **완료** | Phase 2-16에서 snprintf를 mutex 외부로 이동, 잔여 비용 ~40μs/frame |
| 3 | 20 | 세그먼트 검색 최적화 | **보류** | fastDistanceMetersPrecomp() 적용 후 풀스캔 ~120μs — 병목 아님 |
| 3 | 21 | 적응적 디스플레이 리프레시 | **보류** | 별도 기능으로 분류, 전력 최적화 단계에서 구현 |
| 3 | 22 | 컴파일러 경고 전부 수정 | **완료** | wifi_portal.cpp httpd_uri 초기화, dns_server.h ip 초기화 |
| GPS | 5b | UART baud 9600→115200 | **이미 구현됨** | ublox_gps.cpp: UBLOX_BAUD_INIT=9600→UBLOX_BAUD_TARGET=115200 |
| GPS | 5c | CFG-RATE 10Hz 설정 | **이미 구현됨** | configureUBloxModule(): CFG-RATE-MEAS=100ms + Automotive + ACK 확인 |
| GPS | 5d | newData 판정 (iTOW 비교) | **이미 구현됨** | s_newFrameParsed 플래그로 해결 (iTOW 비교 불필요) |
| 문서 | DOC-01~07 | 문서 동기화 | **완료** | hardware.md GPIO, architecture.md, data-structures.md, roadmap, status, changelog, debugging.md |

---

## 결론

이 프로젝트는 **아키텍처와 모듈 구조가 잘 설계**되어 있으며, 핵심 타이밍 알고리즘은 정확합니다. 주요 개선 영역은:

1. ~~**전력 관리가 가장 시급**~~ → **Phase 0에서 해결 완료** (PM DFS, Tickless Idle, WiFi 라이프사이클)
2. ~~**GPS 하드웨어 경로 보완**~~ → **이미 구현됨** (Baud 115200, CFG-RATE 10Hz, Automotive, newData 플래그, 필터/DR 연결)
3. ~~**데드 코드 정리**~~ → **Phase 1에서 해결 완료** (sector.cpp/h 삭제, 미구현 선언 제거, -277줄)
4. ~~**핫패스 미세 최적화**~~ → **Phase 1에서 일부 해결** (LVGL 타임아웃, 시간 업데이트 1Hz)
5. ~~**아키텍처 정리**~~ → **Phase 2에서 해결 완료** (FinishLineDefinition 리네임, lvgl_mutex 경합 분석/최적화, sdkconfig.prod)
6. ~~**컴파일러 경고**~~ → **Phase 3에서 해결 완료** (프로젝트 코드 0 warnings, ESP-IDF 내부 경고만 잔존)
7. **하드웨어 활용** — UART 인터럽트, ULP, 하드웨어 타이머 등 ESP32-S3 기능 미활용. (중장기)
8. ~~**문서 동기화**~~ → **완료** (DOC-01~07 전부 반영)

**남은 중장기 과제:** UART 이벤트 큐 (#15), iTOW 기반 랩타이밍 (HIGH-05), ULP/하드웨어 타이머 활용
