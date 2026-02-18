# GPS 랩 타이머 아키텍처

**상태:** 2026-02-19 기준 최신
**SSOT:** 시스템 아키텍처의 단일 정보원

## 1. 시스템 개요

```
┌─────────────────────────────────────────────────────────────┐
│                         부팅 시퀀스                           │
├─────────────────────────────────────────────────────────────┤
│ 1. 시리얼 초기화 (USB 콘솔, VFS stdin/stdout)                │
│ 2. 저장소 초기화 (NVS + SPIFFS + SD 카드)                     │
│ 3. 디스플레이 초기화 (AXS15231B + LVGL)                      │
│ 4. GPS 초기화 (u-blox G10A-F33, UART2)                      │
│ 5. 레퍼런스 랩 로드                                           │
│ 6. 섹터 타이밍 초기화                                         │
│ 7. 모드 초기화 (기본값: GPSMode::SIMULATION)                 │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                        메인 루프                              │
├─────────────────────────────────────────────────────────────┤
│  ┌────────────────┐        ┌─────────────────┐             │
│  │ GPS_HARDWARE   │   OR   │   SIMULATION    │             │
│  │ processReal    │        │   processSim    │             │
│  └────────────────┘        └─────────────────┘             │
│          │                          │                       │
│          └──────────┬───────────────┘                       │
│                     ▼                                       │
│         ┌───────────────────────┐                           │
│         │ handleSerialCommands  │                           │
│         └───────────────────────┘                           │
│                     │                                       │
│                     ▼                                       │
│         ┌───────────────────────┐                           │
│         │   displayLoop(30Hz)   │                           │
│         └───────────────────────┘                           │
└─────────────────────────────────────────────────────────────┘
```

## 2. 모듈 구조

### 2.1 핵심 모듈 (`main/`)
| 모듈 | 책임 |
|--------|----------------|
| `main.cpp` | 앱 진입점, 메인 루프, 전역 컨텍스트 (`gApp`) |
| `waveshare_display.cpp/h` | 디스플레이 드라이버, LVGL 통합, UI 렌더링 |
| `ublox_gps.cpp/h` | GPS 드라이버 (UBX 프로토콜, UART2) |
| `finish_line.cpp/h` | 결승선 통과 감지 (segment intersection + heading 검증) |
| `lap_storage.cpp/h` | 랩 데이터 영속성 (SD 우선, SPIFFS 폴백) |
| `serial_commands.cpp/h` | CLI 명령 핸들러 |

### 2.2 컴포넌트 (`components/`)

#### Geo (`components/geo/`)
| 파일 | 목적 |
|------|---------|
| `geo_utils.cpp/h` | Haversine 거리, 방향, 투영, `fastDistanceMeters()` |
| `gps_filter.cpp/h` | 스파이크 감지, 스무딩 필터 |
| `dead_reckoning.cpp/h` | GPS 신호 손실 시 위치 추정 |

#### Track (`components/track/`)
| 파일 | 목적 |
|------|---------|
| `track_types.h` | 트랙/레이아웃/섹터 데이터 구조 (`FinishLineDefinition`, `SectorBoundary` 등) |
| `builtin_tracks.h` | 빌트인 트랙 데이터 (에버랜드, 인제) — SD JSON으로 오버라이드 가능 |
| `track_manager.cpp/h` | 트랙 선택 및 피니시라인 기반 자동 식별 |

#### Timing (`components/timing/`)
| 파일 | 목적 |
|------|---------|
| `delta_calculator.cpp/h` | 델타 시간 계산, 누적 거리 |
| `sector_timing.cpp/h` | 섹터별 타이밍 및 델타 추적 |
| `lap_manager.cpp/h` | 랩 및 레퍼런스 랩 관리 |

#### Modes (`components/modes/`)
| 파일 | 목적 |
|------|---------|
| `simulation.cpp/h` | 시뮬레이션 모드 (에버랜드 트랙 재생) |
| `gps_processor.cpp/h` | 실제 GPS 하드웨어 모드 처리 |

### 2.3 Common (`components/common/`)
| 파일 | 목적 |
|------|---------|
| `config.h` | 상수 및 설정 (MAX_REFERENCE_POINTS, 타임아웃 등) |
| `types.h` | 공유 데이터 구조 (AppContext, GPSPoint, LapData, DeltaResult) |

## 3. 데이터 흐름

### 3.1 GPS 하드웨어 모드
```
updateUBloxGPS() → s_newFrameParsed (새 NAV-PVT 프레임 판정)
  └→ UBX NAV-PVT 패킷 수신 (115200 baud, 10Hz)
     └→ GPSPoint 생성 (point.set(lat, lng) + speed, heading, iTOW)
        └→ filterGPSPoint() → 스파이크 감지/스무딩 적용
           └→ addPointToRecording()
              └→ checkLineCrossing() (결승선)
                 └→ onLapComplete() 교차 시
              └→ calculateDelta() → DeltaResult
                 └→ checkSectorTransitionByDistance(trackDistanceM)
                    └→ onSectorComplete() / onSectorEntry()
        └→ GPS 신호 손실 시 Dead Reckoning (200ms 이후 시작)
           └→ updateDeadReckoning() → 추정 위치로 디스플레이 갱신
  └→ updateDisplayData()
```

### 3.2 시뮬레이션 모드
```
processSimulation()
  └→ everland_track_data.h에서 simLapPoints[] 로드
     └→ 시간 기반 재생 (50ms 업데이트, 20Hz)
        └→ currentPoint = lapTimeMs로 보간
           └→ calculateDelta()
              └→ checkSectorTransitionByDistance()
                 └→ checkLineCrossing()
                    └→ onSimLapComplete()
                       └→ updateDisplayData()
```

### 3.3 델타 계산 알고리즘
```
calculateDelta(currentPoint, referenceLap):
  1. 가장 가까운 세그먼트 찾기 (연속성 우선 검색)
     - lastValidSegmentIndex에서 ±50 세그먼트 검색
     - 필요 시 전체 트랙 검색으로 폴백

  2. 세그먼트에 현재 위치 투영
     - 수직 거리 < MAX_PROJECTION_DISTANCE_M (50m)

  3. 트랙 거리 계산
     - trackDistanceM = segmentStartDist + (t * segmentLength)

  4. 동일한 트랙 위치에서 레퍼런스 시간 보간
     - refTimeSec = p1.lapTimeMs + t * (p2.lapTimeMs - p1.lapTimeMs)

  5. 델타 계산
     - deltaSeconds = currentLapTime - refTimeSec
     - confidence = exp(-3.0 * distance / 50.0)

  6. 속도 델타 계산
     - 매칭된 위치에서 레퍼런스 속도 보간
     - speedDeltaKmh = currentSpeed - refSpeed
     - 속도 무효 시 미분으로 폴백
```

### 3.4 섹터 타이밍 흐름
```
initializeSimulation()
  └→ updateSectorDistancesFromReference()
     └→ 각 섹터 경계 좌표에 가장 가까운 점 찾기
        └→ 누적 거리를 경계 임계값으로 저장

processSimulation()/processRealGPS():
  └→ calculateDelta() → trackDistanceM
     └→ checkSectorTransitionByDistance(trackDistanceM)
        └→ if trackDistanceM >= sectorBoundaryDist[i]:
           └→ onSectorComplete(i, lapTimeMs, totalDeltaSeconds)
              └→ sectorDeltas[i] = totalDelta - cumulativeDeltaAtExit[i-1]
              └→ onSectorEntry(i+1, lapTimeMs)

디스플레이:
  - 완료된 섹터: sectorDeltas[i] (고정 값)
  - 현재 섹터: totalDelta - completedCumulativeDelta (실시간)
  - 미래 섹터: 숨김
```

## 4. 전역 상태: AppContext

모든 공유 상태는 `struct AppContext gApp`에 통합됨 (`types.h`에 정의):

```cpp
struct AppContext {
    GPSMode currentGpsMode;          // GPS_HARDWARE 또는 SIMULATION
    GPSPoint currentPoint;           // 현재 GPS 위치
    GPSPoint previousPoint;          // 이전 위치 (계산용)
    DeltaResult currentDelta;        // 현재 델타 결과
    LapData referenceLap;            // 비교용 레퍼런스 랩
    uint16_t currentLapNumber;       // 현재 랩 카운트
    uint16_t currentSessionNumber;   // 세션 ID
    uint32_t bestLapTimeMs;          // 베스트 랩 타임
    bool hasValidReferenceLap;       // 레퍼런스 랩 유효 여부
    int lastValidSegmentIndex;       // 세그먼트 검색 연속성
    Top3Lap top3Laps[3];             // 상위 3 랩
    float batteryVoltage/Percent/MinLeft; // 배터리 상태
    bool gpsTimeSet;                 // GPS 시간 동기화 완료
    char phoneNumber[32];            // 전화번호 플레이트 (WiFi 설정)
    // 전체 정의: types.h 참조
};
```

**접근 패턴:**
- 모든 모듈이 `gApp`에 직접 접근 (헤더에서 extern)
- 게터/세터 오버헤드 없음
- 간단하고 효율적

## 5. 디스플레이 아키텍처

**패널:** AXS15231B QSPI
**드라이버:** `main/waveshare_display.cpp`
**UI 프레임워크:** LVGL 9.x

### 5.1 해상도
- **네이티브 (패널):** 172×640 (세로)
- **논리적 (UI):** 640×172 (가로)
- **회전:** 플러시 콜백에서 소프트웨어 회전

### 5.2 플러시 파이프라인
```
LVGL 렌더 → RGB565 버퍼
  └→ 바이트 스왑 (RGB565 → 패널 형식)
     └→ 소프트웨어 회전 (90° 가로)
        └→ DMA 청크 업로드 (패널 API)
           └→ on_color_trans_done 콜백
              └→ 세마포어 해제 (플러시 페이싱)
```

### 5.3 업데이트 속도
- **디스플레이 루프:** 60 Hz (16ms, DISPLAY_UPDATE_INTERVAL_MS)
- **LVGL 틱:** 5ms (LVGL_TICK_PERIOD_MS)
- **시뮬레이션 업데이트:** 20 Hz (50ms, SIM_UPDATE_INTERVAL_MS)

## 6. 시리얼 콘솔

**경로:** USB 콘솔 (VFS stdin/stdout)
**보드레이트:** 해당 없음 (USB CDC)
**핸들러:** `main/serial_commands.cpp`

**USB 콘솔을 사용하는 이유:**
- UART0는 ESP-IDF 로깅과 충돌
- USB Serial/JTAG는 ESP32-S3 표준
- `idf.py monitor`와 즉시 작동

## 7. 주요 알고리즘

### 7.1 결승선 감지 + 트랙 자동 식별
**방법:** 선 세그먼트 교차
**파일:** `main/finish_line.cpp`

```
결승선: 세그먼트 AB
GPS 트랙: 세그먼트 (previousGPS → currentGPS)

if segmentsIntersect(AB, prev→curr):
  AND 방향이 유효 범위 내 (에버랜드는 315°~45°)
  AND lapTime > MIN_LAP_TIME (30초)
  AND 데드 존 외부 (distance > DEAD_ZONE_M):
    → 랩 완료!
```

**세션 상태머신 (2단계):**
```
PRE_TRACK → (속도 조건 + 피니시라인 정방향 통과) → SESSION_ACTIVE
```
- PRE_TRACK에서 모든 등록 트랙(빌트인 + SD JSON)의 피니시라인을 매 GPS 업데이트마다 체크
- 통과한 피니시라인으로 어떤 트랙인지 자동 식별 (별도 proximity 감지 없음)
- 속도 조건(`SESSION_START_MIN_SPEED_KMH`) 유지

**트랙 정의 우선순위:**
1. SD 카드 JSON (`/sdcard/tracks/<id>.json`) — 있으면 우선
2. 빌트인 (`builtin_tracks.h`) — SD 없거나 해당 트랙 JSON 없을 때 폴백

### 7.2 델타 스무딩
**방법:** 지수 이동 평균 (EMA)
**파일:** `main/main.cpp` → `calculateDelta()`

```cpp
// 시간 델타 스무딩 (alpha = 0.15)
smoothedDeltaSeconds = alpha * rawDelta + (1-alpha) * smoothedDeltaSeconds

// 속도 델타 스무딩 (alpha = 0.20)
smoothedSpeedDelta = alpha * rawSpeedDelta + (1-alpha) * smoothedSpeedDelta
```

### 7.3 GPS 스파이크 필터링
**파일:** `components/geo/gps_filter.cpp`

```
if distance(current, previous) > MAX_JUMP_DISTANCE:
  → 스파이크로 거부
else if moving:
  → 가중 스무딩 적용 (지수 가중치)
```

## 8. 성능 최적화

2026-02-08에 구현됨:

1. **빠른 거리 계산:**
   - `fastDistanceMeters()`: float 전용, double 없음
   - `fastDistanceMetersPrecomp()`: 사전 계산된 cos(lat)
   - `sqrt()` 대신 `sqrtf()` 사용

2. **섹터 타이밍:**
   - `getCumulativeAtSector()`: O(n) → O(1) 직접 조회
   - 레퍼런스 로드 시 경계 거리 캐시

3. **메모리 관리:**
   - `simLapPoints[]`: 정적 배열 (힙 단편화 없음)
   - `LapData` 벡터: 초기화 시 `reserve(MAX_REFERENCE_POINTS)`

4. **GPS 읽기:**
   - UART: 64바이트 청크 읽기 (이전 1바이트)

5. **수학 최적화:**
   - `pow(0.7, i)` → `weight *= 0.7f` (곱셈 루프)

## 9. 검증 체크포인트

아키텍처 변경 시 확인:

- [ ] 빌드 성공 (`idf.py build`)
- [ ] 부팅 로그 표시:
  - 디스플레이 초기화 성공
  - LVGL 초기화됨
  - 첫 플러시 로그
  - `[SectorTiming] Initialized`
  - 섹터 경계 로그
- [ ] 모드가 의도한 기본값 표시 (SIMULATION)
- [ ] 시뮬레이션 중 섹터 감지 로그 표시
- [ ] SSOT 위반 없음 (`docs/README.md`로 확인)

## 참조

- **하드웨어 스펙:** [hardware.md](hardware.md)
- **UI 레이아웃:** [ui-specification.md](ui-specification.md)
- **데이터 구조:** [data-structures.md](data-structures.md)
- **빌드 환경:** [build-environment.md](build-environment.md)
- **트랙 데이터:** [tracks/everland.md](tracks/everland.md)
