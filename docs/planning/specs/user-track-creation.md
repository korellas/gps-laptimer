# User Track Creation (사용자 트랙 생성)

**상태:** 📋 계획됨
**Phase:** 4.7 (데이터 및 연결성)
**필요성:** 높음
**작업량:** 대 (3-4주)
**최종 업데이트:** 2026-02-18

---

## 1. 개요

사용자가 **직접 트랙을 정의**하여, 내장 트랙에 없는 장소에서도 랩타임 측정이 가능하도록 하는 기능.
서킷(순환) 또는 원웨이(직선/구간) 두 가지 트랙 타입을 지원한다.

**왜 필요한가:**
- 내장 트랙 목록에 없는 서킷/장소에서 즉시 사용 가능
- 해외 서킷, 비공식 트랙, 임시 코스, 카트장 등 대응
- 원웨이 모드로 힐클라임, 드래그 스트립, 고갯길 구간 측정 가능
- 사용자 확장성 = 제품 수명 연장 (내장 트랙 추가를 기다릴 필요 없음)

---

## 2. 트랙 타입

### 2.1 서킷 (Circuit)

순환 코스. 출발점과 도착점이 동일.

```
        ┌──────────────┐
        │              │
   ─────┤  서킷 코스   ├─────
  START │              │ = FINISH
  /FINISH└──────────────┘
```

**특성:**
- **시작점 = 결승선**: 1개의 결승선만 정의
- 결승선을 통과할 때마다 랩 완료
- 기존 내장 트랙과 동일한 로직 사용
- 섹터 분할 가능 (선택사항)

### 2.2 원웨이 (One-Way / Point-to-Point)

시작점과 끝점이 다른 구간 코스.

```
  START ──────────────────────────── FINISH
    A          구간 코스              B
```

**특성:**
- **시작점 ≠ 끝점**: 2개의 라인 정의 필요
- 시작 라인 통과 시 타이머 시작, 끝 라인 통과 시 타이머 종료
- 왕복 불가 (방향성 있음 — heading 검증)
- 힐클라임, 드래그, 와인딩 로드 구간 측정에 적합
- 섹터 분할 가능 (선택사항)

---

## 3. 트랙 생성 방법

### 3.1 방법 1: GPS 현장 캡처 (디바이스)

트랙 현장에서 **현재 GPS 위치를 캡처**하여 시작/끝점을 설정.

```
트랙 생성 워크플로우:

1. Settings → "새 트랙 만들기" 진입
2. 트랙 이름 입력 (on-screen keyboard 또는 WiFi 웹 UI)
3. 타입 선택: [서킷] / [원웨이]
4. 결승선 설정:
   - 서킷: "시작/결승선 위치로 이동 → [현재 위치 캡처]"
   - 원웨이: "시작선 위치 → [캡처]" → "끝선 위치 → [캡처]"
5. 방향 설정: 현재 heading을 유효 방향으로 자동 설정 (±45°)
6. (선택) 섹터 포인트 추가
7. 저장 → 트랙 목록에 추가
```

**결승선 캡처 세부사항:**
- 현재 GPS 좌표를 중심으로 **도로 폭 방향 수직선** 자동 생성
- 기본 결승선 폭: 20m (설정 가능)
- heading 기반으로 도로 방향 추정 → 수직 방향으로 결승선 양 끝점 계산:
  ```
  heading = 현재 GPS heading (진행 방향)
  perpendicular = heading ± 90°
  point_A = 현재위치 + bearing(perpendicular, 10m)
  point_B = 현재위치 + bearing(perpendicular+180°, 10m)
  ```

### 3.2 방법 2: WiFi 웹 UI (설정 페이지)

WiFi AP 접속 후 **웹 브라우저에서 트랙 설정**.

```
웹 UI 페이지 구조:

/settings/tracks              → 트랙 목록
/settings/tracks/new          → 새 트랙 생성
/settings/tracks/{id}/edit    → 트랙 수정
/settings/tracks/{id}/delete  → 트랙 삭제
```

**웹 UI 기능:**
- 트랙 이름, 타입, 국가 입력
- GPS 좌표 직접 입력 (위도/경도)
- 또는 **지도에서 클릭** (임베디드 지도 — Leaflet.js + OpenStreetMap)
- 결승선 양 끝점 드래그로 위치/각도 조정
- 섹터 경계 추가/제거
- 실시간 미리보기 (트랙 위에 결승선/섹터 오버레이)
- JSON 형식으로 내보내기/가져오기

### 3.3 방법 3: JSON 파일 직접 업로드

고급 사용자를 위한 JSON 트랙 파일 직접 업로드.

```json
{
  "id": "my_track_001",
  "name": "내 트랙",
  "type": "circuit",
  "country": "KR",
  "center": { "lat": 37.2948, "lng": 127.2021 },
  "detectionRadius": 2000,
  "finishLine": {
    "lat1": 37.29481, "lng1": 127.20210,
    "lat2": 37.29485, "lng2": 127.20225,
    "headingMin": 315, "headingMax": 45
  },
  "sectors": [
    {
      "name": "S1",
      "boundaryLat1": 37.29520, "boundaryLng1": 127.20180,
      "boundaryLat2": 37.29525, "boundaryLng2": 127.20195,
      "headingMin": 0, "headingMax": 90
    }
  ]
}
```

---

## 4. 설정 페이지 (WiFi 웹 UI)

### 4.1 트랙 목록 페이지

```
┌────────────────────────────────────────────────────────────┐
│  🏁 트랙 관리                                    [+ 새 트랙]│
│                                                             │
│  ┌─ 내장 트랙 ────────────────────────────────────────────┐ │
│  │ 🔒 에버랜드 스피드웨이    Full / Short        KR       │ │
│  │ 🔒 인제 스피디움          Full / South / North  KR     │ │
│  └────────────────────────────────────────────────────────┘ │
│                                                             │
│  ┌─ 사용자 트랙 ──────────────────────────────────────────┐ │
│  │ 📝 내 서킷 트랙          서킷    2 섹터   [편집][삭제] │ │
│  │ 📝 고갯길 구간           원웨이   3 섹터   [편집][삭제] │ │
│  └────────────────────────────────────────────────────────┘ │
│                                                             │
│  트랙 파일: [JSON 가져오기]  [전체 내보내기]                 │
└────────────────────────────────────────────────────────────┘
```

### 4.2 트랙 생성/편집 페이지

```
┌────────────────────────────────────────────────────────────┐
│  트랙 설정                                       [저장][취소]│
│                                                             │
│  기본 정보                                                   │
│  ├─ 트랙 이름: [____________________]                       │
│  ├─ 트랙 타입: (●) 서킷  (○) 원웨이                         │
│  └─ 국가:      [KR ▼]                                       │
│                                                             │
│  트랙 중심 (자동 감지용)                                     │
│  ├─ 위도:  [37.294831  ]  경도: [127.202156  ]              │
│  └─ 감지 반경: [2000] m                                     │
│                                                             │
│  결승선 (시작/끝 라인)                                       │
│  ├─ Point A: (37.29481, 127.20210)  [📍 현재 위치]          │
│  ├─ Point B: (37.29485, 127.20225)  [📍 현재 위치]          │
│  ├─ 유효 Heading: [315]° ~ [45]°                            │
│  └─ 결승선 폭: 20m (자동 계산됨)                             │
│                                                             │
│  ── 원웨이 전용 ──                                           │
│  끝 라인                                                     │
│  ├─ Point A: (37.30120, 127.21050)  [📍 현재 위치]          │
│  ├─ Point B: (37.30125, 127.21065)  [📍 현재 위치]          │
│  └─ 유효 Heading: [90]° ~ [180]°                            │
│                                                             │
│  섹터 설정                                    [+ 섹터 추가]  │
│  ├─ S1: (37.29520, 127.20180) → (37.29525, 127.20195)      │
│  ├─ S2: (37.29610, 127.20320) → (37.29615, 127.20335)      │
│  └─ [총 2개 섹터]                                           │
│                                                             │
│  예상 랩 타임                                                │
│  ├─ 예상 시간: [90] 초                                       │
│  ├─ 최소 유효: [30] 초   최대 유효: [300] 초                 │
│  └─ (실제 주행 후 자동 조정 권장)                             │
│                                                             │
│  ┌─ 미리보기 지도 ────────────────────────────────────────┐ │
│  │         🟢 START/FINISH                                │ │
│  │           ╲                                             │ │
│  │     S1 ────╳──── S2                                    │ │
│  │               ╲                                         │ │
│  │         (트랙 중심 기준 지도)                            │ │
│  └────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────┘
```

---

## 5. 데이터 구조

### 5.1 사용자 트랙 정의

기존 `TrackDefinition` 구조체를 확장하여 사용자 트랙도 동일한 인터페이스로 관리.

```cpp
// 트랙 소스 구분
enum class TrackSource {
    BUILTIN,      // 펌웨어 내장 (const, Flash)
    USER_CREATED  // 사용자 생성 (SD/SPIFFS, 런타임 로드)
};

// 트랙 타입 구분
enum class TrackType {
    CIRCUIT,      // 순환 서킷 (시작 = 끝)
    ONE_WAY       // 원웨이 (시작 ≠ 끝)
};

// 사용자 트랙 확장 구조체 (기존 TrackDefinition과 호환)
struct UserTrackDefinition {
    char id[32];              // "user_001"
    char name[64];            // "내 서킷"
    char country[4];          // "KR"

    TrackType type;           // CIRCUIT 또는 ONE_WAY
    TrackSource source;       // USER_CREATED

    // 트랙 중심 (자동 감지용)
    double centerLat;
    double centerLng;
    float detectionRadiusM;

    // 결승선 (서킷: 시작=끝, 원웨이: 시작)
    FinishLineDefinition startLine;

    // 끝 라인 (원웨이 전용, 서킷에서는 미사용)
    FinishLineDefinition endLine;

    // 섹터
    uint8_t sectorCount;
    Sector sectors[MAX_SECTORS_PER_LAYOUT];

    // 예상 랩 타임 (유효성 검증용)
    uint32_t expectedLapTimeMs;
    uint32_t minLapTimeMs;
    uint32_t maxLapTimeMs;

    // 트랙 길이 (첫 주행 후 자동 계산)
    float trackLengthM;

    // 메타데이터
    uint32_t createdTimestamp;    // 생성 시각
    uint32_t modifiedTimestamp;   // 수정 시각
    uint16_t totalRuns;           // 총 주행 횟수
    uint32_t bestTimeMs;          // 베스트 기록
};
```

### 5.2 저장 형식

**파일 경로:**
```
/sd/tracks/user/           (SD 카드)
  ├── user_001.json
  ├── user_002.json
  └── ...

/spiffs/tracks/user/       (SPIFFS 폴백)
  └── user_001.json
```

**JSON 형식:**
```json
{
  "version": 1,
  "id": "user_001",
  "name": "내 서킷",
  "type": "circuit",
  "country": "KR",
  "center": { "lat": 37.2948, "lng": 127.2021 },
  "detectionRadius": 2000,
  "startLine": {
    "lat1": 37.29481, "lng1": 127.20210,
    "lat2": 37.29485, "lng2": 127.20225,
    "headingMin": 315, "headingMax": 45
  },
  "endLine": null,
  "sectors": [
    {
      "name": "S1",
      "lat1": 37.29520, "lng1": 127.20180,
      "lat2": 37.29525, "lng2": 127.20195,
      "headingMin": 0, "headingMax": 90
    }
  ],
  "timing": {
    "expectedMs": 90000,
    "minMs": 30000,
    "maxMs": 300000
  },
  "meta": {
    "created": 1708243200,
    "modified": 1708243200,
    "totalRuns": 0,
    "bestTimeMs": 0
  }
}
```

### 5.3 config.h 추가 상수

```cpp
// User Track
constexpr int MAX_USER_TRACKS = 20;                        // 최대 사용자 트랙 수
constexpr int MAX_TOTAL_TRACKS = 30;                       // 내장 + 사용자 합계
constexpr float DEFAULT_FINISH_LINE_WIDTH_M = 20.0f;       // 결승선 기본 폭
constexpr float USER_TRACK_DEFAULT_DETECTION_M = 2000.0f;  // 기본 감지 반경
constexpr float USER_TRACK_MIN_DETECTION_M = 500.0f;       // 최소 감지 반경
constexpr float USER_TRACK_MAX_DETECTION_M = 10000.0f;     // 최대 감지 반경
```

---

## 6. 원웨이 모드 세부 설계

### 6.1 상태 머신

기존 서킷 모드의 상태 머신을 확장:

```
OneWayState:
  WAITING → ARMED → RUNNING → COMPLETE → WAITING
```

| 상태 | 설명 | 전환 조건 |
|------|------|----------|
| **WAITING** | 시작 라인 근처 대기 | 시작 라인 접근 (proximity) |
| **ARMED** | 시작 준비 완료 | 시작 라인 통과 (line crossing + heading) |
| **RUNNING** | 구간 측정 중 | 끝 라인 통과 |
| **COMPLETE** | 결과 표시 | 터치/타임아웃 → WAITING |

### 6.2 시작/끝 라인 교차 감지

기존 `checkLineCrossing()` 로직 재사용:
- 시작 라인: `startLine` 의 좌표 + heading 범위로 교차 감지
- 끝 라인: `endLine` 의 좌표 + heading 범위로 교차 감지
- 방향 검증으로 역주행 방지

### 6.3 원웨이 특이 사항

| 항목 | 서킷과의 차이 |
|------|-------------|
| 레퍼런스 랩 | 시작→끝 경로 기록 (루프 아님) |
| 델타 계산 | 동일 알고리즘 (트랙 거리 기반) |
| 랩 번호 | "Run #N"으로 표시 (Lap 대신) |
| 자동 재시작 | 시작 라인으로 돌아가야 다음 런 가능 |
| 섹터 | 시작→끝 사이 구간 분할 (동일) |

---

## 7. 트랙 자동 감지 통합

사용자 트랙도 내장 트랙과 동일한 자동 감지 파이프라인에 포함:

```
GPS fix 획득
  └→ trackManager.detectTrack(lat, lng)
     ├→ 내장 트랙 검색 (builtin_tracks[])
     └→ 사용자 트랙 검색 (user_tracks[])  ← 추가
        └→ haversine(current, track.center) < track.detectionRadius
           └→ 가장 가까운 트랙 선택
              └→ 타입에 따라 서킷/원웨이 모드 자동 전환
```

**감지 우선순위:**
1. 내장 트랙 (정확도 높음, 검증된 데이터)
2. 사용자 트랙 (거리순)
3. 감지 반경 겹침 시 가장 가까운 트랙 우선

---

## 8. 검증 규칙

트랙 저장 시 유효성 검사:

| 검증 항목 | 규칙 | 오류 메시지 |
|----------|------|-----------|
| 트랙 이름 | 1~64자, 비어있지 않음 | "트랙 이름을 입력하세요" |
| 결승선 좌표 | 유효한 위도(-90~90), 경도(-180~180) | "유효하지 않은 좌표입니다" |
| 결승선 폭 | 5~100m | "결승선 폭이 너무 좁거나 넓습니다" |
| 원웨이 끝 라인 | type=ONE_WAY일 때 필수 | "원웨이 트랙은 끝 라인이 필요합니다" |
| 시작/끝 거리 | 원웨이: 두 라인 간 100m 이상 | "시작과 끝이 너무 가깝습니다" |
| 감지 반경 | 500~10,000m | "감지 반경 범위를 확인하세요" |
| 섹터 순서 | 시작→끝 방향으로 순차적 | "섹터 순서가 올바르지 않습니다" |
| ID 중복 | 기존 트랙과 ID 중복 불가 | "이미 존재하는 트랙 ID입니다" |
| 최소 랩 타임 | > 10초 | "최소 랩 타임이 너무 짧습니다" |

---

## 9. 의존성

| 의존 항목 | Phase | 필수 여부 |
|----------|-------|----------|
| 다중 트랙 지원 | Phase 2.1-2.2 | 필수 |
| 트랙 자동 감지 | Phase 2.2 | 필수 |
| 터치 UI 설정 메뉴 | Phase 3.1 | 높음 |
| WiFi 웹 서버 | Phase 4.2 | 높음 (웹 UI) |
| 트랙 설정 웹 인터페이스 | Phase 4.3 | 높음 (확장) |
| SD 카드 저장 | Phase 4.1 | 선택 (SPIFFS 폴백) |

---

## 10. 구현 우선순위

1. **P1**: `UserTrackDefinition` 구조체 + JSON 파서
2. **P1**: 사용자 트랙 로드/저장 (SPIFFS/SD)
3. **P1**: 트랙 자동 감지에 사용자 트랙 통합
4. **P1**: 서킷 타입 — 기존 로직과 동일, 즉시 작동
5. **P2**: 원웨이 타입 — 시작/끝 라인 분리 로직
6. **P2**: 디바이스 GPS 캡처 (현재 위치로 결승선 설정)
7. **P2**: WiFi 웹 UI — 트랙 목록/생성/편집/삭제
8. **P3**: 웹 UI 지도 통합 (Leaflet.js)
9. **P3**: JSON 가져오기/내보내기
10. **P3**: 섹터 설정 UI

---

## 참조

- **기존 트랙 구조:** [../../reference/tracks/README.md](../../reference/tracks/README.md)
- **트랙 타입 정의:** `components/track/track_types.h`
- **features-backlog:** [../features-backlog.md](../features-backlog.md)
- **roadmap:** [../roadmap.md](../roadmap.md) Phase 4.7
- **Phase 4.3 웹 인터페이스:** 이 기능의 상위 인프라
