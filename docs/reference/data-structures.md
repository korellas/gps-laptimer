# 데이터 구조

**상태:** 2026-02-14 기준 최신
**SSOT:** 핵심 데이터 구조의 단일 정보원

## 1. 개요

모든 핵심 데이터 구조는 `components/common/types.h`에 정의되어 있습니다.

## 2. 핵심 구조

### 2.1 GPSPoint

단일 GPS 측정 포인트를 나타냅니다.

```cpp
struct GPSPoint {
    double lat = 0.0;               // 위도 (도, WGS84)
    double lng = 0.0;               // 경도 (도, WGS84)
    unsigned long gpsTimeMs = 0;    // GPS 타임스탬프 (ms) — GPS 모드에서 iTOW 저장
    unsigned long lapTimeMs = 0;    // 현재 랩 내 시간 (ms)
    float speedKmh = 0.0f;          // 속도 (km/h)
    float headingDeg = 0.0f;        // 방향 (도, 0-359, 0=북쪽)
    bool initialized = false;       // 유효 데이터 설정 여부

    bool isValid() const;           // initialized 기반 유효성 체크
    void set(double lat, double lng); // 좌표 설정 + initialized=true
    void clear();                   // 모든 필드 초기화
};
```

**사용법:**
- 현재 위치: `gApp.currentPoint`
- 이전 위치: `gApp.previousPoint`
- 레퍼런스 랩 포인트: `gApp.referenceLap.points[]`

**좌표:**
- 형식: WGS84 십진 도
- 정밀도: double (~1cm 정확도를 위해 일반적으로 소수점 7-8자리)
- 예시: `lat=37.296096, lng=127.206860`

### 2.2 LapData

메타데이터가 포함된 완전한 랩 기록을 나타냅니다.

```cpp
struct LapData {
    std::vector<GPSPoint> points;           // 랩 중 기록된 GPS 포인트
    std::vector<float> cumulativeDistances; // 각 포인트의 누적 거리 (미터)
    float totalTrackDistance;               // 총 트랙 길이 (미터)
    unsigned long totalTimeMs;              // 총 랩 타임 (밀리초)
    unsigned long startTimeMs;              // 랩 시작 타임스탬프 (GPS 시간)
    float maxSpeedKmh;                      // 랩 중 최대 속도
    float avgSpeedKmh;                      // 평균 속도
};
```

**주요 필드:**

| 필드 | 설명 | 예시 |
|------|------|------|
| `points` | 순차 GPS 측정값 | ~1분 랩의 경우 500-1500 포인트 |
| `cumulativeDistances` | 시작부터 각 포인트까지 거리 | [0, 15.2, 32.7, ...] 미터 |
| `totalTrackDistance` | 전체 랩 거리 | 12881.0 미터 (에버랜드) |
| `totalTimeMs` | 랩 완료 시간 | 52830 ms (52.83초) |

**메모리:**
- `MAX_REFERENCE_POINTS = 1600` (config.h)
- 초기화 시 `reserve(MAX_REFERENCE_POINTS)`로 벡터 사전 할당
- 힙 단편화 방지를 위한 시뮬레이션 데이터 정적 할당

### 2.3 DeltaResult

레퍼런스 랩 대비 델타 계산 결과.

```cpp
struct DeltaResult {
    float deltaSeconds;      // 시간 델타 (+: 느림, -: 빠름)
    float speedDeltaKmh;     // 매칭된 위치에서 속도 델타
    float distanceMeters;    // 트랙까지 수직 거리 (m)
    float confidence;        // 매칭 신뢰도 (0.0-1.0)
    int refPointIndex;       // 매칭된 레퍼런스 세그먼트 인덱스
    float trackDistanceM;    // 시작부터 트랙을 따라 거리 (m)
    float refTimeSec;        // 이 위치의 레퍼런스 랩 타임 (s)
};
```

**계산 흐름:**
```
calculateDelta() → DeltaResult
  ├─ deltaSeconds: currentLapTime - refTimeSec
  ├─ speedDeltaKmh: currentSpeed - 보간된 refSpeed
  ├─ trackDistanceM: 매칭된 포인트까지 누적 거리
  ├─ confidence: exp(-3.0 * distanceMeters / 50.0)
  └─ refPointIndex: 레퍼런스 랩에서 매칭된 세그먼트 인덱스
```

**사용법:**
- 저장 위치: `gApp.currentDelta`
- 업데이트: `main/main.cpp`의 `calculateDelta()`
- 디스플레이: 중앙 델타 텍스트, 속도 바

### 2.4 AppContext

전역 애플리케이션 상태 (싱글톤 `gApp`).

```cpp
struct AppContext {
    // GPS 모드
    GPSMode currentGpsMode = GPSMode::SIMULATION;

    // 레퍼런스 랩
    LapData referenceLap;
    bool hasValidReferenceLap = false;

    // 현재 GPS 위치
    GPSPoint currentPoint;
    GPSPoint previousPoint;

    // 현재 델타
    DeltaResult currentDelta;

    // 랩/세션 추적
    uint16_t currentLapNumber = 1;
    uint16_t currentSessionNumber = 1;
    uint32_t bestLapTimeMs = UINT32_MAX;

    // 세그먼트 검색 연속성
    int lastValidSegmentIndex = -1;

    // 상위 3 랩 (랩 완료 시에만 갱신)
    struct Top3Lap {
        uint16_t lapNumber = 0;
        uint32_t lapTimeMs = UINT32_MAX;
    };
    Top3Lap top3Laps[3];

    // 배터리 모니터링
    float batteryVoltage = 0.0f;    // 배터리 전압 (V)
    float batteryPercent = -1.0f;   // SoC (%), -1 = 미측정
    float batteryMinLeft = -1.0f;   // 남은 시간 (분), -1 = 추정 불가

    // GPS 시간 동기화
    bool gpsTimeSet = false;

    // WiFi Portal 설정
    char phoneNumber[32] = "";      // 전화번호 플레이트

    // 헬퍼
    bool isSimulationMode() const;
};
```

**접근 패턴:**
- 전역 인스턴스: `extern AppContext gApp;`
- 모든 모듈이 직접 접근 (getter/setter 없음)
- 정의 위치: `components/common/types.h`
- 인스턴스 위치: `components/common/app_context.cpp`

### 2.5 GPSMode

작동 모드 열거형.

```cpp
enum class GPSMode {
    GPS_HARDWARE,  // 실제 GPS 모듈 (u-blox)
    SIMULATION     // 시뮬레이션 재생 (에버랜드 트랙 데이터)
};
```

**기본 모드:** `GPSMode::SIMULATION`

**전환:**
- 시리얼 명령: `e` (모드 토글)
- 함수: `main/serial_commands.cpp`의 `toggleGPSMode()`

## 3. 섹터 타이밍 구조

### 3.1 CurrentSectorTiming

현재 랩 섹터 진행 상황 추적.

```cpp
struct CurrentSectorTiming {
    int currentSector;                      // 현재 섹터 인덱스 (0부터 시작)
    float sectorDeltas[MAX_SECTORS];        // 각 완료된 섹터의 델타
    float cumulativeDeltaAtExit[MAX_SECTORS]; // 섹터 종료 시 총 델타
    unsigned long sectorEntryTimes[MAX_SECTORS]; // 섹터 진입 시 랩 타임
    bool sectorCompleted[MAX_SECTORS];      // 완료 플래그
};
```

**정의 위치:** `components/timing/sector_timing.h`
**최대 섹터 수:** `MAX_SECTORS = 8` (config.h)

**계산:**
```
onSectorComplete(i, lapTime, totalDelta):
  cumulativeDeltaAtExit[i] = totalDelta
  sectorDeltas[i] = totalDelta - cumulativeDeltaAtExit[i-1]
  sectorCompleted[i] = true
```

### 3.2 섹터 경계 데이터

레퍼런스 랩을 따라 거리로 저장된 섹터 경계.

```cpp
// components/timing/sector_timing.cpp
static float s_sectorBoundaryDistances[MAX_SECTORS];
```

**초기화:**
- 트랙 섹터 경계 좌표 (lat/lng)에서 계산
- 레퍼런스 랩에서 가장 가까운 포인트 찾기
- 해당 포인트의 누적 거리 저장
- 레퍼런스 랩 변경 시 업데이트

## 4. 트랙 구조

### 4.1 SectorBoundary

섹터 경계 좌표 정의.

```cpp
struct SectorBoundary {
    double lat;       // 경계 포인트 위도
    double lng;       // 경계 포인트 경도
    const char* name; // 섹터 이름 (예: "S1→S2")
};
```

**정의 위치:** `components/track/track_types.h`

### 4.2 TrackLayout

완전한 트랙 정의.

```cpp
struct TrackLayout {
    const char* name;                  // 트랙 이름
    double finishLat1, finishLng1;     // 결승선 포인트 A
    double finishLat2, finishLng2;     // 결승선 포인트 B
    float finishHeadingMin;            // 유효 방향 범위 최소값 (도)
    float finishHeadingMax;            // 유효 방향 범위 최대값 (도)
    int numSectors;                    // 섹터 수
    SectorBoundary sectorBoundaries[MAX_SECTORS]; // 섹터 경계
};
```

**정의 위치:** `components/track/track_types.h`
**내장 트랙:** `components/track/builtin_tracks.h`

## 5. 상수

`components/common/config.h`의 주요 상수:

```cpp
// 용량
constexpr int MAX_REFERENCE_POINTS = 1600;     // 레퍼런스 랩의 최대 포인트 수
constexpr int MAX_SECTORS_PER_LAYOUT = 8;      // 트랙당 최대 섹터 수

// 임계값
constexpr float MAX_PROJECTION_DISTANCE_M = 50.0f; // 트랙 매칭 최대 거리
constexpr unsigned long GPS_TIMEOUT_MS = 3000;      // GPS 신호 타임아웃
constexpr unsigned long MIN_LAP_TIME_MS = 30000;    // 최소 유효 랩 타임 (30초)

// 업데이트 간격
constexpr unsigned long SIM_UPDATE_INTERVAL_MS = 50;        // 시뮬레이션 (20 Hz)
constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 16;    // 디스플레이 (60 Hz)

// 섹터 타이밍
constexpr unsigned long SECTOR_DELTA_DISPLAY_MS = 5000;     // 섹터 델타 표시 시간
constexpr float SECTOR_PROGRESS_MAX = 2.0f;                 // 진행률 클램프 최대값
```

## 6. 타입 규칙

| 타입 | 사용 용도 | 예시 |
|------|----------|------|
| `double` | 지리적 좌표만 | `lat`, `lng` |
| `float` | 기타 모든 부동소수점 | `speedKmh`, `deltaSeconds` |
| `unsigned long` | 타임스탬프 (밀리초) | `gpsTimeMs`, `lapTimeMs` |
| `int` | 카운터, 인덱스 | `currentSector`, `lapNumber` |
| `bool` | 플래그 | `isRecording`, `gpsSignalLost` |

**근거:**
- 좌표에 `double`: ~1cm 정밀도 유지
- 기타에 `float`: 성능 최적화, 충분한 정밀도
- 참조: `docs/development/decisions/003-float-optimization.md`

## 7. 메모리 관리

### 7.1 힙 할당
```cpp
// LapData 벡터 (힙, 하지만 사전 예약)
referenceLap.points.reserve(MAX_REFERENCE_POINTS);
referenceLap.cumulativeDistances.reserve(MAX_REFERENCE_POINTS);
```

### 7.2 정적 할당
```cpp
// 시뮬레이션 데이터 (힙 단편화 방지)
static GPSPoint simLapPoints[MAX_REFERENCE_POINTS];
```

### 7.3 스택 할당
```cpp
// 임시 계산 (로컬 스코프)
DeltaResult calculateDelta(...) {
    DeltaResult result;  // 스택 할당
    // ...
    return result;
}
```

## 8. 직렬화

### 8.1 SPIFFS 저장 형식

SPIFFS에 바이너리 파일로 저장된 랩 데이터:

```
/laps/session_<N>_lap_<M>.dat
```

**형식:** `LapData` 구조의 바이너리 덤프
**함수:** `main/lap_storage.cpp`의 `saveLap()`, `loadLap()`

## 9. 검증

데이터 구조 수정 시 확인사항:

- [ ] 크기가 메모리에 맞는지 (`sizeof()`로 확인)
- [ ] 정렬이 올바른지 (특히 DMA/플래시의 경우)
- [ ] 초기화되지 않은 필드 없음
- [ ] `float` vs `double` 일관성 있게 사용
- [ ] 벡터에 `reserve()` 호출 있음
- [ ] SSOT: 구조당 하나의 정의만 존재

## 참조

- **아키텍처:** [architecture.md](architecture.md)
- **하드웨어:** [hardware.md](hardware.md)
- **트랙 데이터:** [tracks/everland.md](tracks/everland.md)
- **소스 코드:** `components/common/types.h`, `components/common/config.h`
