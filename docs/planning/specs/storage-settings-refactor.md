# 저장소 및 설정 리팩토링 스펙

**상태:** 계획됨 (Phase 2)
**작성일:** 2026-02-18
**우선순위:** P1 (필수)

---

## 1. 배경

현재 SPIFFS에 4개 파일이 분산 저장되어 있고, 트랙 정의가 펌웨어에 하드코딩되어 있다.
이로 인해:
- 트랙 추가 시 펌웨어 빌드 필요
- 설정이 여러 파일/포맷(바이너리, JSON)에 흩어져 있음
- SPIFFS 2MB 용량 한계로 랩 데이터 수십 개면 꽉 참
- NEAR_TRACK proximity detection이 불필요한 오버헤드

## 2. 현재 상태

### SPIFFS 파일 목록

| 경로 | 내용 | 포맷 | 사용 빈도 |
|------|------|------|----------|
| `/spiffs/config/settings.json` | 전화번호 | JSON | 부팅 1회 읽기, 설정 변경 시 쓰기 |
| `/spiffs/config/finish_line.bin` | 수동 피니시라인 좌표 | 바이너리 (struct dump) | **사실상 미사용** |
| `/spiffs/config/imu_fusion.json` | IMU 축 캘리브레이션 | JSON | 부팅 읽기, 주행 중 업데이트 |
| `/spiffs/laps/` | 랩 데이터 | 바이너리 | 랩 완료 시 쓰기, 세션 시작 시 읽기 |

### 트랙 정의 (하드코딩)

`components/track/builtin_tracks.h`에 에버랜드/인제 트랙이 C++ 코드로 하드코딩:
- 트랙 중심 좌표, 감지 반경
- 피니시라인 좌표 (A, B), 유효 heading 범위
- 섹터 boundary 좌표
- 기대/최소/최대 랩타임, 트랙 길이

### 세션 상태머신 (현재 3단계)

```
PRE_TRACK → (proximity 감지) → NEAR_TRACK → (속도 + 라인통과) → SESSION_ACTIVE
```

- PRE_TRACK: 모든 트랙 중심점까지 bbox+haversine 거리 체크
- NEAR_TRACK: 피니시라인 로드, 속도 60km/h 이상에서 라인 통과 대기
- SESSION_ACTIVE: 랩 타이밍

## 3. 변경 계획

### 3.1 finish_line.bin 삭제

**이유:** 빌트인 트랙이 있으면 `setFinishLineFromDefinition()`이 트랙 정의에서 직접 로드.
SPIFFS의 finish_line.bin은 시리얼 `SET_FL` 수동 설정에서만 사용되며, 사실상 dead path.

**삭제 대상:**
- `finish_line.cpp`: `loadFinishLineFromStorage()`, `saveFinishLineToStorage()` 바이너리 I/O
- `finish_line.cpp`: `FINISH_LINE_FILE`, `FINISH_LINE_DIR` 상수
- `finish_line.cpp`: `migrateLegacyFinishLine()` 레거시 마이그레이션

**유지:**
- `setFinishLineFromDefinition()` — 트랙 정의에서 로드 (핵심 경로)
- `setFinishLineFromCurrentPos()` — 수동 설정 시 settings.json에 저장하도록 변경
- `checkLineCrossing()`, `checkFirstLineCrossing()` — 교차 감지 로직

### 3.2 설정 통합 (settings.json 확장)

현재 `{"phone":"010xxxx"}`만 있는 settings.json을 통합 설정 파일로 확장.

```json
{
  "phone": "01012345678",
  "deltaRef": "today",
  "customFinishLine": null
}
```

| 필드 | 타입 | 설명 | 기본값 |
|------|------|------|--------|
| `phone` | string | 전화번호판 | `""` |
| `deltaRef` | string | 델타 비교 기준: `"today"` / `"session"` | `"today"` |
| `customFinishLine` | object/null | 사용자 수동 피니시라인 (null이면 트랙 정의 사용) | `null` |

`customFinishLine` 구조 (설정 시):
```json
{
  "lat1": 37.xxx, "lng1": 127.xxx,
  "lat2": 37.xxx, "lng2": 127.xxx,
  "bearingMin": 80, "bearingMax": 100
}
```

**변경 안 하는 것:**
- `/spiffs/config/imu_fusion.json` — 내부 전용, 사용자에게 노출할 필요 없음. SPIFFS 유지.

### 3.3 트랙 정의: JSON 파일 + 빌트인 폴백

#### 파일 위치
- SD: `/sdcard/tracks/<track_id>.json` (우선)
- 빌트인: `builtin_tracks.h` (SD 없거나 해당 트랙 JSON 없을 때 폴백)

#### JSON 포맷

```json
{
  "id": "inje",
  "name": "Inje Speedium",
  "country": "KR",
  "layouts": [
    {
      "id": "full",
      "name": "Full Course",
      "finishLine": {
        "A": [38.000583, 128.291111],
        "B": [38.000561, 128.291517],
        "headingRange": [340, 60]
      },
      "timing": {
        "minLapTimeMs": 60000,
        "maxLapTimeMs": 300000
      },
      "trackLengthM": 3908,
      "sectors": [
        {
          "name": "S1",
          "boundary": {
            "A": [38.004285, 128.293848],
            "B": [38.004088, 128.294081],
            "headingRange": [20, 75]
          }
        },
        {
          "name": "S2",
          "boundary": {
            "A": [37.998836, 128.291779],
            "B": [37.998758, 128.292106],
            "headingRange": [170, 225]
          }
        },
        {
          "name": "S3",
          "boundary": {
            "A": [38.000031, 128.285964],
            "B": [37.999769, 128.285887],
            "headingRange": [255, 315]
          }
        }
      ]
    }
  ]
}
```

#### 설계 원칙

- **트랙 중심 좌표**: JSON에 포함하지 않음. 피니시라인 A, B 두 점의 중앙으로 자동 계산.
- **`expectedLapTimeMs` 제거**: 코드에서 참조하지 않는 dead field.
- **빌트인 폴백**: `builtin_tracks.h`는 유지. SD에 같은 id의 JSON이 있으면 오버라이드.
- **감지 반경(`detectionRadiusM`)**: proximity detection 제거로 불필요해짐. JSON에 포함하지 않음.

### 3.4 세션 상태머신 단순화 (3단계 → 2단계)

**변경 후:**

```
PRE_TRACK → (속도 조건 + 피니시라인 정방향 통과) → SESSION_ACTIVE
```

**제거 대상:**
- `GPSSessionState::NEAR_TRACK` 상태
- `updateTrackProximity()` — bbox + haversine 근접 감지
- `track_manager.cpp`의 proximity 관련 로직
- `TRACK_PROXIMITY_HYSTERESIS_FACTOR` 상수

**PRE_TRACK 동작 변경:**
- 모든 등록된 트랙(빌트인 + SD JSON)의 피니시라인을 매 GPS 업데이트마다 체크
- 트랙 2~3개 × segment intersection = 비용 무시 가능 (10Hz)
- 피니시라인 통과 시 해당 트랙 자동 식별 → SESSION_ACTIVE 진입
- 속도 조건(`SESSION_START_MIN_SPEED_KMH`) 유지

**트랙 식별:**
- 어떤 트랙의 피니시라인을 통과했는지로 자동 식별
- 별도의 proximity/거리 기반 감지 불필요
- 통과한 피니시라인의 트랙 + 레이아웃 정보로 섹터/레퍼런스 로드

### 3.5 `expectedLapTimeMs` 제거

`track_types.h`의 `TrackLayout.expectedLapTimeMs` 필드는 코드 어디에서도 참조되지 않음.
`builtin_tracks.h`에서 값만 정의되어 있고 읽는 곳이 없는 dead code.

**삭제:**
- `track_types.h`: `expectedLapTimeMs` 필드 제거
- `builtin_tracks.h`: 에버랜드/인제 해당 값 제거

## 4. 영향받는 파일

| 파일 | 변경 내용 |
|------|----------|
| `main/finish_line.cpp` | 바이너리 I/O 제거, SPIFFS 경로/마이그레이션 제거 |
| `main/finish_line.h` | 스토리지 관련 함수 선언 제거 |
| `components/modes/gps_processor.cpp` | NEAR_TRACK case 제거, PRE_TRACK에서 전체 피니시라인 체크 |
| `components/track/track_manager.cpp/h` | proximity 로직 제거, JSON 트랙 로더 추가 |
| `components/track/track_types.h` | `expectedLapTimeMs` 제거 |
| `components/track/builtin_tracks.h` | `expectedLapTimeMs` 제거, `trackLengthM` 정정 |
| `components/wifi_portal/wifi_portal.cpp` | `loadSettings()`/`saveSettings()` 확장 |
| `components/common/types.h` | `GPSSessionState::NEAR_TRACK` 제거, deltaRef 설정 추가 |
| `components/common/config.h` | proximity 관련 상수 제거 |
| **신규**: 트랙 JSON 로더 | SD에서 트랙 JSON 파싱, 빌트인 오버라이드 |

## 5. 마이그레이션

### 기존 사용자 데이터
- `finish_line.bin` → 삭제 (트랙 정의에서 자동 로드되므로 데이터 손실 없음)
- `settings.json` → 기존 `{"phone":"..."}` 포맷 하위 호환. 새 필드는 없으면 기본값 사용.
- `imu_fusion.json` → 변경 없음
- `/spiffs/laps/` → 변경 없음

### SD 카드 트랙 파일
- 초기에는 SD에 트랙 JSON 없어도 빌트인으로 동작
- 사용자가 트랙 추가/수정하고 싶을 때만 SD에 JSON 생성

## 6. 구현 순서

1. `expectedLapTimeMs` 제거 (가장 단순, 위험 낮음)
2. `finish_line.bin` 바이너리 I/O 제거
3. `settings.json` 확장 (deltaRef, customFinishLine)
4. NEAR_TRACK 제거 + 상태머신 단순화
5. 트랙 JSON 로더 구현 + SD 파일 오버라이드
6. `builtin_tracks.h` 값 정정 (trackLengthM 등)
