# GPS 하드웨어 구현 계획

**작성일:** 2026-02-16
**상태:** 구현 예정
**브랜치:** `refactor/code-review-cleanup`

## 개요

시뮬레이션 모드는 완전히 동작하지만, GPS 하드웨어 모드는 부분 구현 상태.
UBX NAV-PVT 파서와 기본 데이터 흐름은 있으나, 모듈 설정(baud/rate), 섹터 타이밍, GPS 필터링 등 핵심 기능이 누락되어 있다.

**목표:** GPS 모드를 시뮬레이션 모드와 동등한 수준으로 끌어올림.

## 현재 상태

### 동작하는 것
- UBX NAV-PVT 바이너리 파싱 (`main/ublox_gps.cpp`)
- UART2 초기화 (GPIO44/43, 64바이트 청크 읽기)
- 기본 GPS 모드 흐름 (`components/modes/gps_processor.cpp`): GPS → GPSPoint → delta → display
- GPS 신호 유실 감지 (3초 타임아웃)
- GPS 시작 화면 (위성 수, fix 상태)
- 피니시라인 감지 (세그먼트 교차 + heading 검증)
- 랩 SPIFFS 저장

### 문제점
| 문제 | 심각도 | 파일 |
|------|--------|------|
| 9600 baud 고정 → 10Hz 불가 | P0 | `ublox_gps.cpp`, `config.h` |
| 모듈에 설정 명령 미전송 (1Hz 기본값) | P0 | `ublox_gps.cpp` |
| `newData` 감지 버그 (중복 처리) | P0 | `ublox_gps.cpp:191-204` |
| 섹터 타이밍 완전 부재 | P1 | `gps_processor.cpp` |
| `GPSPoint.isValid()` 항상 false | P1 | `gps_processor.cpp:95-101` |
| 디스플레이 GPS fix 도착 시에만 갱신 | P1 | `gps_processor.cpp` |
| GPS 필터/DR 미연결 | P1 | `gps_processor.cpp` |
| 베스트 랩 후 `referenceLap` 미갱신 | P1 | `main.cpp` |

---

## Phase 1: UBX 모듈 설정 (P0 - 최우선)

**문제:** 현재 9600 baud 고정, 모듈에 설정 명령 전송 없음.
공장 기본값은 1Hz NMEA+UBX 혼합 → 10Hz 불가능.

### 작업

1. **UBX 명령 전송 인프라** — `main/ublox_gps.cpp`
   - `sendUbxMessage(class, id, payload, len)` 함수 추가
     - Fletcher-8 체크섬 계산 (기존 `calcUbxChecksum` 재활용)
     - `uart_write_bytes()` 전송
   - `sendCfgValset(keyValues[], count, layers)` 헬퍼
     - M10은 **CFG-VALSET(0x06/0x8A)만 지원**, 레거시 CFG-PRT/CFG-RATE 불가
     - layers: RAM(0x01) 또는 RAM+BBR(0x03)

2. **부트 설정 시퀀스** — `configureUBloxModule()` 신규 함수
   ```
   1. UART2 9600 초기화 (현재 코드 유지)
   2. 100ms 대기
   3. CFG-VALSET → UART1 baudrate = 115200
   4. 100ms 대기 + UART RX 버퍼 flush
   5. UART2 재초기화 115200 (uart_driver_delete → uart_driver_install)
   6. 50ms 대기
   7. CFG-VALSET → RATE-MEAS=100ms(10Hz), NAV-PVT 활성, NMEA 비활성, DYNMODEL=Automotive
   8. UBX-ACK 대기 및 확인
   ```

3. **ACK/NAK 파싱** — `parseUbxMessage()`에 추가
   - UBX-ACK-ACK (0x05/0x01)
   - UBX-ACK-NAK (0x05/0x00)
   - 설정 성공/실패 판단에 사용

4. **config.h 상수 업데이트**
   ```cpp
   // 변경:
   constexpr int GPS_BAUD_RATE_INIT = 9600;      // 공장 기본값
   constexpr int GPS_BAUD_RATE_TARGET = 115200;   // 운용 baud rate
   constexpr int GPS_UPDATE_RATE_HZ = 10;
   constexpr int GPS_MEAS_PERIOD_MS = 100;
   ```

### CFG-VALSET 키 ID (M10 SPG 5.10)

> **주의:** 아래 키 ID는 u-blox M10 인터페이스 문서에서 정확한 값을 확인해야 함

| Key ID | 이름 | 크기 | 값 | 용도 |
|--------|------|------|-----|------|
| `0x40520001` | CFG-UART1-BAUDRATE | U4 | 115200 | UART baud rate |
| `0x30210001` | CFG-RATE-MEAS | U2 | 100 | 측정 주기 100ms |
| `0x30210002` | CFG-RATE-NAV | U2 | 1 | 측정당 1 nav 솔루션 |
| `0x20910007` | CFG-MSGOUT-UBX_NAV_PVT_UART1 | U1 | 1 | NAV-PVT 활성 |
| `0x209100BB` | CFG-MSGOUT-NMEA_ID_GGA_UART1 | U1 | 0 | NMEA GGA 비활성 |
| `0x209100CA` | CFG-MSGOUT-NMEA_ID_GLL_UART1 | U1 | 0 | NMEA GLL 비활성 |
| `0x209100C0` | CFG-MSGOUT-NMEA_ID_GSA_UART1 | U1 | 0 | NMEA GSA 비활성 |
| `0x209100C5` | CFG-MSGOUT-NMEA_ID_GSV_UART1 | U1 | 0 | NMEA GSV 비활성 |
| `0x209100AC` | CFG-MSGOUT-NMEA_ID_RMC_UART1 | U1 | 0 | NMEA RMC 비활성 |
| `0x209100B1` | CFG-MSGOUT-NMEA_ID_VTG_UART1 | U1 | 0 | NMEA VTG 비활성 |
| `0x20110021` | CFG-NAVSPG-DYNMODEL | E1 | 4 | Automotive 모드 |

### 수정 파일
- `main/ublox_gps.cpp` — sendUbxMessage, sendCfgValset, configureUBloxModule, ACK 파싱
- `main/ublox_gps.h` — 새 함수 선언
- `components/common/config.h` — baud rate 상수
- `main/main.cpp` — app_init()에서 configureUBloxModule() 호출

### 검증
- NAV-PVT 10Hz 수신 확인 (시리얼 로그)
- NMEA 텍스트 미수신 확인
- 시리얼에 "Baud=115200" 출력

---

## Phase 2: newData 감지 버그 수정 (P0)

**문제:** `updateUBloxGPS()`가 첫 3D fix 이후 매 호출마다 `true` 반환.
`lastData.valid`가 영구적 플래그라서 한번 `true`되면 계속 `true`.

**위치:** `main/ublox_gps.cpp:191-204`

### 수정
```cpp
static bool s_newFrameParsed = false;

// parseUbxMessage()에서 NAV-PVT 파싱 성공 시:
s_newFrameParsed = true;

// updateUBloxGPS():
bool updateUBloxGPS() {
    s_newFrameParsed = false;  // 시작 시 리셋
    // ... UART 읽기 루프 ...
    return s_newFrameParsed;   // 새 프레임 파싱된 경우에만 true
}
```

### 수정 파일
- `main/ublox_gps.cpp`

### 검증
- `processRealGPS()` 호출 빈도 ~10Hz 확인 (5초마다 카운터 출력)

---

## Phase 3: GPS 모드 초기화 (P0)

**문제:** GPS 모드 진입 시 트랙 활성화, 섹터 타이밍, 레퍼런스 랩 로딩 없음.
`main.cpp:725-733`에서 시뮬레이션만 초기화하고 GPS는 빈 상태로 진입.

### 작업
`initializeGPSMode()` 함수를 `gps_processor.cpp`에 구현:

```cpp
void initializeGPSMode() {
    initializeGPSProcessor();                    // 기존 GPS 상태 리셋
    setActiveTrackById("everland", "full");       // 트랙 활성화
    initSectorTiming();                          // 섹터 타이밍 초기화
    initGPSFilter();                             // GPS 필터 초기화
    initDeadReckoning();                         // Dead Reckoning 초기화
    // 레퍼런스 랩은 app_init()의 loadReferenceLapFromStorage()에서 이미 로딩
    if (gApp.hasValidReferenceLap) {
        updateSectorDistancesFromReference(...);  // 섹터 경계 거리 계산
        resetSectorTiming();
    }
}
```

`main.cpp` main_task에서:
```cpp
if (!modeInitialized) {
    if (gApp.isSimulationMode()) {
        initializeSimulation();
    } else {
        initializeGPSMode();   // 추가
    }
    modeInitialized = true;
}
```

### 수정 파일
- `components/modes/gps_processor.cpp` — initializeGPSMode() 구현
- `components/modes/gps_processor.h` — 함수 선언
- `main/main.cpp` — GPS 모드 초기화 호출

### 검증
- 시리얼에 "[GPS] Track activated", "[SectorTiming] Initialized" 출력

---

## Phase 4: GPSPoint.isValid() 수정 (P1)

**문제:** GPS 포인트가 필드별 할당으로 `initialized=false` 유지 → `isValid()` 항상 `false`.
피니시라인 감지(`previousPoint.isValid()`)와 필터링에 영향.

**위치:** `components/modes/gps_processor.cpp:95-101`

### 수정
```cpp
GPSPoint point;
point.set(ubx.lat, ubx.lon);        // initialized = true 자동 설정
point.speedKmh = ubx.speedKmh;
point.headingDeg = ubx.headingDeg;
point.gpsTimeMs = ubx.iTOW;         // GPS 시간 활용 (기존: 0)
point.lapTimeMs = lapTimeMs;
```

### 수정 파일
- `components/modes/gps_processor.cpp`

---

## Phase 5: 섹터 타이밍 추가 (P1)

**문제:** GPS 모드에 섹터 감지 코드 완전 부재.
시뮬레이션은 `checkSectorTransitionByDistance()` 매 프레임 호출.

### 작업
`processRealGPS()`의 delta 계산 후에 추가:
```cpp
if (gApp.hasValidReferenceLap && gApp.currentDelta.trackDistanceM >= 0) {
    int completedSector = checkSectorTransitionByDistance(
        gApp.currentDelta.trackDistanceM);
    if (completedSector >= 0) {
        onSectorComplete(completedSector, lapTimeMs,
                        gApp.currentDelta.deltaSeconds);
        const auto& st = getCurrentSectorTiming();
        if (completedSector + 1 < st.totalSectors) {
            onSectorEntry(completedSector + 1, lapTimeMs);
        }
    }
}
```

`onLapComplete()`에 추가:
- `resetSectorTiming()`
- 새 베스트 랩 시 `updateSectorDistancesFromReference()`

### 수정 파일
- `components/modes/gps_processor.cpp` — 섹터 감지 로직
- `main/main.cpp` — onLapComplete()에 섹터 리셋

---

## Phase 6: 디스플레이 연속 업데이트 (P1)

**문제:** `updateDisplayData()`가 새 GPS fix 도착 시에만 호출 → 100ms 간격 화면 멈춤.
시뮬레이션은 50ms마다 항상 호출.

### 수정
`processRealGPS()` 끝에서 GPS fix 여부와 무관하게 항상 호출:
```cpp
// 함수 끝:
if (gpsState.lapStarted) {
    unsigned long lapTimeMs = now - gpsState.lapStartMs;
    updateDisplayData(gApp.currentPoint, gApp.currentDelta,
                     lapTimeMs, gApp.currentDelta.refTimeSec);
}
```

### 수정 파일
- `components/modes/gps_processor.cpp`

---

## Phase 7: 레퍼런스 랩 메모리 갱신 (P1)

**문제:** 새 베스트 랩 SPIFFS 저장 후 `gApp.referenceLap` 미갱신 → 이후 랩 델타 0.00 고정.

### 수정
`onLapComplete()`에서 `saveBestLap()` 후:
```cpp
LapData newRef;
if (convertStorableToLapData(completedLap, newRef)) {
    setReferenceLap(newRef);
    calculateCumulativeDistances(gApp.referenceLap);
    updateSectorDistancesFromReference(...);
}
```

### 수정 파일
- `main/main.cpp` — onLapComplete()

---

## Phase 8: GPS 필터 연결 (P1)

**문제:** `gps_filter.cpp`(스파이크 감지 + 스무딩) 완성되었으나 `processRealGPS()`에서 호출 안 됨.

### 작업
delta 계산 전에 필터 통과:
```cpp
FilteredGPSPoint filtered = filterGPSPoint(point, now);
if (filtered.wasSpikeFiltered) {
    point = filtered.point;  // 마지막 유효 위치 사용
} else if (filtered.isValid) {
    point = filtered.point;  // 스무딩된 위치 사용
}
```

### 수정 파일
- `components/modes/gps_processor.cpp`
- include 추가: `#include "../geo/gps_filter.h"`

---

## Phase 9: Dead Reckoning 연결 (P2)

**문제:** `dead_reckoning.cpp` 완성되었으나 미연결.

### 작업
GPS 신호 200ms 이상 두절 시 DR 시작:
```cpp
if (!gotNewGPSData && lapStarted) {
    if (elapsed > 200 && !isDeadReckoningActive()) {
        startDeadReckoning(currentPoint, speedKmh, headingDeg, now);
    }
    if (isDeadReckoningActive() && !isDeadReckoningExpired()) {
        GPSPoint estimated = updateDeadReckoning(now);
        updateDisplayData(estimated, ...);
    }
}
```
GPS 복귀 시 `stopDeadReckoning()`.

### 수정 파일
- `components/modes/gps_processor.cpp`
- include 추가: `#include "../geo/dead_reckoning.h"`

---

## Phase 10: GPS 진단 시리얼 명령 (P2)

시리얼 명령 `g` 추가:
```
=== GPS Status ===
Fix: 3D (8 satellites)
Pos: 37.296178, 127.213808
Speed: 82.3 km/h  Heading: 342.1
iTOW: 123456789 ms
Baud: 115200, Rate: 10 Hz
Filter: 0.2% spike rate
DR: inactive
```

### 수정 파일
- `main/main.cpp` (serial command handler)

---

## 구현 순서

| 순서 | Phase | 우선순위 | 예상 시간 | 마일스톤 |
|------|-------|---------|----------|---------|
| 1 | Phase 1: UBX 모듈 설정 | P0 | 3-4h | |
| 2 | Phase 2: newData 버그 수정 | P0 | 30m | |
| 3 | Phase 3: GPS 모드 초기화 | P0 | 1h | **최소 동작 GPS** |
| 4 | Phase 4: isValid() 수정 | P1 | 15m | |
| 5 | Phase 5: 섹터 타이밍 | P1 | 1h | |
| 6 | Phase 6: 디스플레이 업데이트 | P1 | 30m | |
| 7 | Phase 7: 레퍼런스 랩 갱신 | P1 | 30m | |
| 8 | Phase 8: GPS 필터 연결 | P1 | 1h | **시뮬레이션 동등** |
| 9 | Phase 9: Dead Reckoning | P2 | 1h | |
| 10 | Phase 10: 진단 명령 | P2 | 1h | **강건성 + 도구** |

**총 예상: ~10-12시간**

---

## 검증 계획

### Phase 1-3 완료 후 (실내 테스트)
- GPS 모듈 연결, 실외/창가에서 위성 수신 확인
- 10Hz NAV-PVT 수신 확인 (시리얼 로그)
- 기본 랩 타이밍 동작 확인

### Phase 4-8 완료 후 (트랙 테스트)
- 에버랜드 서킷 실주행
- 섹터 델타 표시 확인
- 피니시라인 감지 및 랩 저장 확인
- GPS 스파이크 필터링 동작 확인

### 각 Phase마다
- `ninja` 빌드 성공 확인
- 시뮬레이션 모드 기존 동작 미영향 확인

---

## 참고: 주요 파일 맵

| 파일 | 역할 |
|------|------|
| `main/ublox_gps.cpp/h` | UBX 프로토콜 드라이버 |
| `components/modes/gps_processor.cpp/h` | GPS 하드웨어 모드 루프 |
| `components/modes/simulation.cpp` | 참조 구현 (모든 기능 동작) |
| `components/timing/sector_timing.cpp/h` | 섹터 타이밍 |
| `components/geo/gps_filter.cpp/h` | GPS 스파이크 필터 |
| `components/geo/dead_reckoning.cpp/h` | Dead Reckoning |
| `components/common/config.h` | 상수 정의 |
| `main/main.cpp` | 메인 루프, updateDisplayData(), onLapComplete() |
