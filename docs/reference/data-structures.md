# 데이터 구조

**상태:** 2026-02-19 기준 최신 (코드 기준 v1.1.0)
**SSOT:** 핵심 데이터 구조의 단일 정보원

## 1. 개요

모든 핵심 데이터 구조는 `components/common/types.h`에 정의되어 있습니다.
트랙/레이아웃 구조는 `components/track/track_types.h`, 섹터 타이밍은 `components/timing/sector_timing.h`, 랩 저장소는 `main/lap_storage.h`에 있습니다.

## 2. 핵심 구조

### 2.1 GPSPoint

단일 GPS 측정 포인트를 나타냅니다.

```cpp
struct GPSPoint {
    double lat = 0.0;               // 위도 (도, WGS84)
    double lng = 0.0;               // 경도 (도, WGS84)
    unsigned long gpsTimeMs = 0;    // GPS 타임스탬프 (ms) — iTOW
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

### 2.2 LapData

메타데이터가 포함된 완전한 랩 기록을 나타냅니다.

```cpp
struct LapData {
    std::vector<GPSPoint> points;           // 랩 중 기록된 GPS 포인트
    std::vector<float> cumulativeDistances; // 각 포인트의 누적 거리 (미터)
    float totalTrackDistance = 0.0f;        // 총 트랙 길이 (미터)
    unsigned long totalTimeMs = 0;          // 총 랩 타임 (밀리초)
    unsigned long startTimeMs = 0;          // 랩 시작 타임스탬프
    float maxSpeedKmh = 0.0f;               // 랩 중 최대 속도
    float avgSpeedKmh = 0.0f;              // 평균 속도
};
```

**메모리:**
- `MAX_REFERENCE_POINTS = 1600` (config.h)
- 초기화 시 `reserve(MAX_REFERENCE_POINTS)`로 벡터 사전 할당

### 2.3 DeltaResult

레퍼런스 랩 대비 델타 계산 결과.

```cpp
struct DeltaResult {
    float deltaSeconds = 0.0f;      // 시간 델타 (+: 느림, -: 빠름)
    float distanceMeters = FLT_MAX; // 트랙까지 수직 거리 (m)
    float confidence = 0.0f;        // 매칭 신뢰도 (0.0-1.0)
    int refPointIndex = -1;         // 매칭된 레퍼런스 세그먼트 인덱스
    float trackDistanceM = 0.0f;    // 시작부터 트랙을 따라 거리 (m)
    float refTimeSec = 0.0f;        // 이 위치의 레퍼런스 랩 타임 (s)
    float refSpeedKmh = 0.0f;       // 이 위치의 레퍼런스 속도 (km/h)
    float speedDeltaKmh = 0.0f;     // 속도 델타 (+: 레퍼런스보다 빠름, -: 느림)
    bool hasSpeedDelta = false;     // 속도 델타 유효 여부
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

### 2.4 AppContext

전역 애플리케이션 상태 (싱글톤 `gApp`).

```cpp
struct AppContext {
    // GPS 모드
    GPSMode currentGpsMode = GPSMode::GPS_HARDWARE;  // 기본: GPS 하드웨어

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

    // GPS 시간 동기화
    bool gpsTimeSet = false;

    // SD 카드 상태
    bool sdCardMounted = false;

    // IMU (QMI8658C)
    ImuData imuData;
    ImuCalibration imuCalibration;
    bool imuReady = false;

    // Sensor Fusion (GPS+IMU)
    float fusedSpeedKmh = 0.0f;      // 칼만 필터 출력 속도 (km/h)
    bool  fusionActive = false;       // 퓨전 동작 중 (칼만 필터 초기화됨)
    bool  fusionInDR = false;         // GPS 없이 predict만 진행 중
    float lastGpsHeadingRad = 0.0f;   // 마지막 GPS heading (forward 투영용, rad)
    uint32_t fusionCalibDoneMs = 0;   // 캘리브레이션 완료 시각 (UI 알림용)

    // WiFi Portal 설정
    char phoneNumber[32] = "";      // 전화번호 플레이트

    // OTA 상태
    OTAState otaState = OTAState::IDLE;
    float otaProgress = 0.0f;          // 0.0 - 1.0
    uint32_t otaReceivedBytes = 0;
    uint32_t otaTotalBytes = 0;
    char otaErrorMsg[64] = "";

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
    GPS_HARDWARE,  // 실제 GPS 모듈 (u-blox) — 기본값
    SIMULATION     // 시뮬레이션 재생 (에버랜드 트랙 데이터)
};
```

**기본 모드:** `GPSMode::GPS_HARDWARE`

### 2.6 ImuData / ImuCalibration

```cpp
struct ImuData {
    float accelX, accelY, accelZ;       // 캘리브 후 가속도 (g)
    float rawAccelX, rawAccelY, rawAccelZ; // 원시 가속도 (g)
    float gyroX, gyroY, gyroZ;          // 자이로 (°/s)
    float temperature;                  // 칩 온도 (°C)
    bool valid;
    uint32_t timestampMs;
};

struct ImuCalibration {
    float accelOffsetX, accelOffsetY, accelOffsetZ;
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    bool calibrated;
    int samplesUsed;
};
```

**정의 위치:** `components/common/types.h`

### 2.7 OTAState

```cpp
enum class OTAState {
    IDLE,       // OTA 미진행
    RECEIVING,  // 데이터 수신 중
    VALIDATING, // 이미지 검증 중
    COMPLETE,   // 완료, 리부트 대기
    ERROR       // 오류 발생
};
```

## 3. 섹터 타이밍 구조

**정의 위치:** `components/timing/sector_timing.h`

### 3.1 CurrentSectorTiming

현재 랩 섹터 진행 상황 추적.

```cpp
struct CurrentSectorTiming {
    unsigned long sectorTimes[MAX_SECTORS_PER_LAYOUT];       // 각 섹터 소요 시간 (ms)
    unsigned long sectorEntryTimes[MAX_SECTORS_PER_LAYOUT];  // 섹터 진입 시 랩 타임
    bool sectorCompleted[MAX_SECTORS_PER_LAYOUT];            // 완료 플래그

    float sectorDeltas[MAX_SECTORS_PER_LAYOUT];              // 섹터별 델타 (초)
    float cumulativeDeltaAtExit[MAX_SECTORS_PER_LAYOUT];     // 섹터 종료 시 총 델타

    int currentSector;         // 현재 섹터 인덱스 (-1 = 미시작)
    int completedCount;        // 완료된 섹터 수
    int totalSectors;          // 총 섹터 수

    // 마지막 완료 섹터 정보 (표시용)
    int lastCompletedSector;
    unsigned long lastSectorTime;
    float lastSectorDelta;
    unsigned long lastSectorCompletedAt;  // 완료 시각 (millis)
};
```

**최대 섹터 수:** `MAX_SECTORS_PER_LAYOUT = 8` (config.h)

**델타 계산:**
```
onSectorComplete(i, lapTime, totalDelta):
  cumulativeDeltaAtExit[i] = totalDelta
  sectorDeltas[i] = totalDelta - cumulativeDeltaAtExit[i-1]
  sectorCompleted[i] = true
```

### 3.2 SectorBoundaryPoint

섹터 경계 GPS 좌표 (거리 계산용).

```cpp
struct SectorBoundaryPoint {
    double lat;
    double lng;
};
```

섹터 경계 거리는 `updateSectorDistancesFromReference()`로 레퍼런스 랩에서 계산됩니다.

## 4. 트랙 구조

**정의 위치:** `components/track/track_types.h`

### 4.1 FinishLineDefinition

결승선 정의 (두 GPS 좌표로 이루어진 선분).

```cpp
struct FinishLineDefinition {
    double lat1, lng1;          // 포인트 A
    double lat2, lng2;          // 포인트 B
    float validHeadingMin;      // 유효 방향 범위 최소 (도, 0-360)
    float validHeadingMax;      // 유효 방향 범위 최대 (도, 0-360)

    bool isConfigured() const;
    void clear();
};
```

### 4.2 Sector

섹터 경계 정의 (경계선 방식 또는 거리 방식).

```cpp
struct Sector {
    const char* name;               // 표시 이름 ("S1", "Sector 1" 등)

    // 경계선 방식 (boundaryLat1 != 0 이면 사용)
    double boundaryLat1, boundaryLng1;
    double boundaryLat2, boundaryLng2;
    float validHeadingMin;
    float validHeadingMax;

    // 거리 방식 (경계선 미사용 시)
    float startDistanceM;           // 트랙 시작부터 거리 (m)
    float endDistanceM;

    bool usesBoundaryLine() const;  // 경계선 방식 여부
    bool usesDistance() const;      // 거리 방식 여부
};
```

### 4.3 TrackLayout

트랙의 특정 레이아웃 구성.

```cpp
struct TrackLayout {
    const char* id;             // 고유 식별자 ("full", "short", "south", "north")
    const char* name;           // 표시 이름 ("Full Course", "Short Course")

    FinishLineDefinition finishLine;  // 이 레이아웃의 결승선

    uint32_t minLapTimeMs;      // 최소 유효 랩 타임 (더 빠르면 무시)
    uint32_t maxLapTimeMs;      // 최대 유효 랩 타임 (더 느리면 무시)

    int sectorCount;            // 섹터 수 (0 = 섹터 없음)
    const Sector* sectors;      // 섹터 배열 (nullptr if sectorCount == 0)

    bool hasBuiltinReference;   // 펌웨어 내장 레퍼런스 데이터 여부
    int builtinRefStartIdx;     // track_points 배열 시작 인덱스
    int builtinRefEndIdx;       // track_points 배열 끝 인덱스
    uint32_t builtinRefTimeMs;  // 내장 레퍼런스 랩 타임

    float trackLengthM;         // 총 트랙 길이 (m)

    bool hasSectors() const;
    bool hasReference() const;
    const Sector* getSector(int index) const;
};
```

### 4.4 TrackDefinition

레이싱 서킷 전체 정의.

```cpp
struct TrackDefinition {
    const char* id;             // 고유 식별자 ("everland", "inje")
    const char* name;           // 표시 이름 ("Everland Speedway")
    const char* country;        // 국가 코드 ("KR", "JP", "US")

    int layoutCount;
    const TrackLayout* layouts;

    const TrackLayout* getLayout(int index) const;
    const TrackLayout* getLayoutById(const char* layoutId) const;
    const TrackLayout* getDefaultLayout() const;
};
```

**내장 트랙:** `components/track/builtin_tracks.h`
- Everland Speedway: `full`, `short` 레이아웃
- Inje Speedium: `full`, `south`, `north` 레이아웃

### 4.5 ActiveTrack

현재 활성화된 트랙/레이아웃 런타임 상태.

```cpp
struct ActiveTrack {
    const TrackDefinition* track;   // 선택된 트랙 (nullptr = 없음)
    const TrackLayout* layout;      // 선택된 레이아웃
    int layoutIndex;
    bool userConfirmed;

    bool isValid() const;
    void clear();
    const char* getTrackName() const;
    const char* getLayoutName() const;
    const FinishLineDefinition* getFinishLineDefinition() const;
    int getSectorCount() const;
};
```

## 5. 저장소 구조

**정의 위치:** `main/lap_storage.h`

### 5.1 파일 형식

랩 데이터는 바이너리 파일로 저장됩니다.

```
파일 레이아웃:
  [LapHeader (36 bytes)] + [StoredPoint × pointCount]
```

```cpp
struct LapHeader {
    uint32_t magic;           // 0x4C415001 ("LAP" + v1)
    uint16_t version;         // v2: trackIndex/layoutIndex 추가
    uint16_t pointCount;
    uint32_t totalTimeMs;
    uint32_t startTimestamp;  // Unix epoch (KST)
    uint16_t maxSpeedX10;
    uint16_t avgSpeedX10;
    uint16_t sessionId;
    uint16_t lapId;
    uint8_t  trackIndex;      // BUILTIN_TRACKS[] 인덱스, 0xFF=미지정
    uint8_t  layoutIndex;     // layouts[] 인덱스, 0xFF=미지정
    uint8_t  reserved[6];
};

struct StoredPoint {
    int32_t lat;          // lat * 1e7 (압축 고정소수)
    int32_t lng;          // lng * 1e7
    uint32_t lapTimeMs;
    uint16_t speedX10;    // speed * 10 (0-6553.5 km/h)
    uint8_t heading;      // 0-255 = 0-360 degrees
};
```

### 5.2 파일 경로

SD 카드가 마운트된 경우 SD, 아니면 SPIFFS 사용:

| 파일 종류 | 경로 |
|-----------|------|
| 일반 랩 | `{lapsDir}/s{sessionId}_l{lapId}.bin` |
| 트랙별 베스트랩 | `{lapsDir}/best_{trackId}_{layoutId}.bin` |

- SD 마운트 시: `lapsDir = /sdcard/laps`
- SPIFFS 사용 시: `lapsDir = /spiffs/laps`

### 5.3 StorableLap (런타임)

```cpp
struct StorableLap {
    std::vector<StoredPoint> points;
    uint32_t totalTimeMs;
    uint32_t startTimestamp;
    float maxSpeedKmh;
    float avgSpeedKmh;
    uint16_t sessionId;
    uint16_t lapId;
    uint8_t trackIndex = 0xFF;
    uint8_t layoutIndex = 0xFF;
};
```

## 6. 상수

`components/common/config.h`의 주요 상수:

```cpp
// 용량
constexpr int MAX_REFERENCE_POINTS  = 1600;    // 레퍼런스 랩 최대 포인트 수
constexpr int MAX_SECTORS_PER_LAYOUT = 8;       // 레이아웃당 최대 섹터 수
constexpr int MAX_POINTS_PER_LAP    = 12000;   // 저장 포인트 안전 상한 (lap_storage.h)

// 임계값
constexpr float MAX_PROJECTION_DISTANCE_M = 50.0f;  // 트랙 매칭 최대 거리
constexpr unsigned long GPS_TIMEOUT_MS    = 3000;   // GPS 신호 타임아웃
constexpr unsigned long MIN_LAP_TIME_MS   = 30000;  // 최소 유효 랩 타임 (30초)
constexpr float SESSION_START_MIN_SPEED_KMH = 60.0f; // 세션 시작 최소 속도
constexpr float AUTO_ENTER_LAPTIMER_SPEED_KMH = 50.0f; // 랩타이머 자동 진입 속도

// 업데이트 간격
constexpr unsigned long SIM_UPDATE_INTERVAL_MS     = 50;  // 시뮬레이션 (20 Hz)
constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 16;  // 디스플레이 (60 Hz)

// 섹터 타이밍
constexpr unsigned long SECTOR_DELTA_DISPLAY_MS = 5000;  // 섹터 델타 표시 시간
constexpr float SECTOR_PROGRESS_MAX = 2.0f;              // 진행률 클램프 최대값
```

## 7. 타입 규칙

| 타입 | 사용 용도 | 예시 |
|------|----------|------|
| `double` | 지리적 좌표만 | `lat`, `lng` |
| `float` | 기타 모든 부동소수점 | `speedKmh`, `deltaSeconds` |
| `unsigned long` | 타임스탬프 (밀리초) | `gpsTimeMs`, `lapTimeMs` |
| `uint32_t` | 저장용 타임스탬프 | `totalTimeMs` in LapHeader |
| `int` | 카운터, 인덱스 | `currentSector`, `lapNumber` |
| `bool` | 플래그 | `isRecording`, `gpsSignalLost` |

## 8. 메모리 관리

### 8.1 힙 할당
```cpp
// LapData 벡터 (힙, 하지만 사전 예약)
referenceLap.points.reserve(MAX_REFERENCE_POINTS);
referenceLap.cumulativeDistances.reserve(MAX_REFERENCE_POINTS);
```

### 8.2 정적 할당
```cpp
// 시뮬레이션 데이터 (힙 단편화 방지)
static GPSPoint simLapPoints[MAX_REFERENCE_POINTS];
```

### 8.3 스택 할당
```cpp
// 임시 계산 (로컬 스코프)
DeltaResult calculateDelta(...) {
    DeltaResult result;  // 스택 할당
    return result;
}
```

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
- **소스 코드:** `components/common/types.h`, `components/common/config.h`, `components/track/track_types.h`, `components/timing/sector_timing.h`, `main/lap_storage.h`
