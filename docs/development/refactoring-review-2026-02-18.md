# GPS Lap Timer 리팩터링 검토 문서

작성일: 2026-02-18  
대상: `main/`, `components/` 전반  
목적: 리팩터링 우선순위 확정 및 실행 기준 정리

## 1. 한눈에 요약
1. 먼저 고쳐야 할 항목은 기능 정확성(PRE_TRACK 시작 판정), OTA 보안, 저장 파일 로드 안전성입니다.
2. 그 다음은 모듈 경계 정리(`components` ↔ `main` 의존), 미사용 코드 정리(`lap_manager`)입니다.
3. 마지막으로 대형 파일 분리(`waveshare_display.cpp`)와 델타 계산 엣지케이스 정리를 권장합니다.

## 2. 리팩터링 항목 (번호별)

### 1. PRE_TRACK 트랙 감지 로직 안정화
- 우선순위: Critical
- 문제: PRE_TRACK 루프에서 매 포인트마다 피니시 라인 설정을 다시 적용하면서 crossing 상태를 리셋해 감지 성공 조건이 깨질 수 있음.
- **[코드 검토 후 업데이트]** 실제 코드 확인 결과 "깨질 수 있음"이 아니라 **구조적으로 항상 실패**하는 상태임. 설계 실수가 아니라 구현 버그.

  **루트 코즈**: `setFinishLineFromDefinition()`이 내부에서 `resetCrossingState()`를 호출하고, 이것이 `hasPrevPoint=false`로 초기화함. PRE_TRACK 루프는 GPS 포인트마다 모든 트랙/레이아웃을 순회하면서 이 함수를 반복 호출하므로, `checkFirstLineCrossing()`이 필요로 하는 "이전 포인트" 누적이 GPS 업데이트를 넘어서 절대 유지되지 않음.

  ```
  GPS N, 이터레이션 T0/L0: setFinishLine → hasPrevPoint=false → checkFirst: "no prev" → 저장만
  GPS N, 이터레이션 T0/L1: setFinishLine → hasPrevPoint=false (리셋!) → checkFirst: "no prev" → 저장만
  GPS N+1, 이터레이션 T0/L0: setFinishLine → hasPrevPoint=false (또 리셋!) → checkFirst: "no prev" → 저장만
  → checkFirstLineCrossing이 true를 리턴하는 경로가 존재하지 않음
  ```

  `startSession()`으로 가는 유일한 경로가 이 `checkFirstLineCrossing` 성공인데, 사실상 세션 자동 시작이 동작하지 않았을 것으로 판단됨. 실주행 테스트 전 확인 필요.

- 근거 파일:
  - `components/modes/gps_processor.cpp` (L310-L343: PRE_TRACK 루프)
  - `main/finish_line.cpp` (L173-L192: `setFinishLineFromDefinition` → `resetCrossingState`)
- 제안:
  - 트랙/레이아웃 후보별로 최초 1회만 `setFinishLineFromDefinition()` 호출
  - 동일 후보에 대해 crossing state 유지
  - 후보 변경 시에만 crossing state reset
- **구현 시 중점 확인 포인트**:
  1. `setFinishLineFromDefinition()` 내부의 `resetCrossingState()` 호출이 SESSION_ACTIVE용 의도임을 확인하고, PRE_TRACK 경로에서는 호출하지 않도록 분리.
  2. 트랙/레이아웃별 crossing state를 배열(`CrossingState s_preTrackState[MAX_TRACK_CANDIDATES]`)로 관리하거나, 마지막으로 체크한 후보를 기억해 동일 후보일 때만 state를 재사용하는 방식 선택.
  3. 후보가 바뀔 때만 state를 리셋하므로, `checkFirstLineCrossing`에서 prevLat/prevLng가 다른 트랙의 finish line 기준으로 누적된 값이 되지 않도록 주의 — 트랙별 좌표 스케일 문제는 없지만, bearing 필터가 다른 finish line의 validBearingMin/Max 기준으로 걸러질 수 있음.
  4. `initializeGPSMode()` / `resetRealGPS()`에서 신규 per-candidate state 배열도 함께 초기화해야 함.
- 기대 효과: 자동 세션 진입 구현 완성 (현재는 동작하지 않음), 실주행 시작 판정 신뢰도 확보

### 2. OTA 경로 보안 강화 (Wi-Fi/BLE 공통)
- 우선순위: High → **[코드 검토 후 재평가] Medium으로 하향 권장**
- 문제: OTA 업로드 경로에 인증/권한 제어가 약하고, BLE OTA도 보안 플래그 없이 쓰기 허용 범위가 큼.
- **[코드 검토 후 업데이트]** 실제 코드 확인 결과, 위험도는 인지되나 **개인 장치 특성**을 고려하면 High 우선순위는 과잉임.

  **WiFi 현황** (`wifi_portal.cpp` L833):
  - SoftAP가 OPEN (인증 없음), 모든 HTTP 엔드포인트에 인증 없음
  - 완화 요인: 사용자가 SETTINGS 모드에서 **수동으로 켜야** 동작, `max_connection=1`로 단일 접속만 허용, 물리적 접근 필요 (~30m 반경 내)
  - 실질적 공격 시나리오: WiFi를 켠 상태에서 30m 내 악의적 접근자가 SSID 발견 후 업로드. 경기장 내에서는 현실적 위협이 될 수 있음.

  **BLE 현황** (`ble_ota.cpp` L344,354,365):
  - `min_key_size = 0`: Control/Data/Status 모두 암호화/페어링 불필요
  - Control(0xFFE1), Data(0xFFE2) 모두 무인증 Write 가능 — WiFi보다 접근이 쉬움 (BLE 스캔으로 바로 발견)
  - BLE OTA가 실제로 더 큰 공격면임

  **권장 조치 (비용 대비 효과 순)**:
  1. WiFi AP에 WPA2 패스워드 추가 — 코드 1줄 변경, 가장 cost-effective
  2. BLE pairing 요구 (`BLE_GATT_CHR_F_WRITE_AUTHEN`) — 중간 수준 변경
  3. 세션 토큰/nonce, 이미지 서명은 개인 장치에 과잉

- 근거 파일:
  - `components/wifi_portal/wifi_portal.cpp` (L833: `WIFI_AUTH_OPEN`)
  - `components/ble_ota/ble_ota.cpp` (L344-365: `min_key_size = 0`)
- 기대 효과: WPA2 패스워드 1줄로 WiFi 공격면 대부분 해소, BLE 패치는 선택적으로 적용

### 3. 저장 파일 로드 시 pointCount 상한 검증
- 우선순위: High (**[코드 검토 후 확인] 그대로 유지**)
- 문제: 손상 파일 헤더의 `pointCount`를 그대로 신뢰해 대규모 할당이 발생할 수 있음.
- **[코드 검토 후 업데이트]** 진단 정확. `loadBestLapFromFile()`과 `loadLap()` 두 곳 모두 동일 패턴.

  **현재 코드** (`main/lap_storage.cpp` L303, L249):
  ```cpp
  lap.points.resize(header.pointCount);  // ← magic 체크 이후지만 pointCount 상한 검증 없음
  rd = fread(lap.points.data(), 1, header.pointCount * sizeof(StoredPoint), f);
  ```
  파일이 손상되어 `header.pointCount = 0xFFFFFFFF`라면 `resize()` 호출이 수백 MB 할당을 시도 → ESP32-S3 내부 RAM 512KB 기준 즉시 실패하거나 힙 손상.
  `magic` 체크는 있으나 `pointCount`는 uint32_t라 magic이 맞아도 값 범위가 무제한.

  **패치 포인트**: `loadBestLapFromFile()` L303 직전, `loadLap()` L249 직전 동일하게 적용:
  ```cpp
  if (header.pointCount == 0 || header.pointCount > MAX_POINTS_PER_LAP) {
      fclose(f);
      return false;
  }
  ```
  파일 크기 검증 (선택적): `fseek(f, 0, SEEK_END); fsize = ftell(f)` 후 `sizeof(LapHeader) + header.pointCount * sizeof(StoredPoint) == fsize` 확인.

- 근거 파일:
  - `main/lap_storage.cpp` (L249: `loadLap`, L303: `loadBestLapFromFile`)
- 기대 효과: 메모리 보호, 비정상 파일 입력 내구성 향상. 패치 범위 최소, 즉시 적용 가능.

### 4. 모듈 경계 정리 (`components`가 `main`을 참조하는 구조)
- 우선순위: Medium (**[코드 검토 후 확인] 그대로 유지, 실제 코드에서 확인됨**)
- 문제: `components/*.h`에서 `../main/*.h`를 include하는 브리지 구조가 증가해 계층이 역전됨.
- **[코드 검토 후 업데이트]** 진단 정확. `components/` 안에 `main/`을 re-export하는 shim 헤더 6개 확인됨:
  ```
  components/finish_line.h      → #include "../main/finish_line.h"
  components/ublox_gps.h        → #include "../main/ublox_gps.h"
  components/protocol.hpp       → #include "../main/protocol.hpp"
  components/waveshare_display.h→ #include "../main/waveshare_display.h"
  components/lap_storage.h      → #include "../main/lap_storage.h"
  components/serial_commands.h  → #include "../main/serial_commands.h"
  ```
  이 shim들은 ESP-IDF CMake include path 우회용으로 생성된 것으로 보임. `components/modes/gps_processor.cpp`에서 `extern void updateDisplayData(...)` 같은 extern 선언으로 `main/` 함수를 직접 참조하는 패턴도 병행 존재 (L38-40).

  **리팩터링 방향**: 두 가지 전략 중 선택 필요.
  - A) `main/` 핵심 헤더들을 `components/common/`으로 이동 — 근본 해결, 파급 범위 큼
  - B) shim 헤더 존재 자체를 허용하되 `extern` 직접 참조만 제거 — 최소 변경

- 근거 파일:
  - `components/*.h` (shim 6개)
  - `components/modes/gps_processor.cpp` (L38-40: extern 직접 참조)
- 기대 효과: 의존성 추적 단순화, 테스트/교체/재사용성 향상

### 5. 미사용/부분구현 모듈 정리 (`lap_manager`)
- 우선순위: Medium (**[코드 검토 후] 선택지 B(삭제)를 적극 권장으로 변경**)
- 문제: API는 많지만 실제 호출이 거의 없고 TODO가 남아 유지보수 비용만 증가.
- **[코드 검토 후 업데이트]** 예상보다 심각. **현재 아키텍처에서 `lap_manager`는 실제 코드 경로에서 전혀 사용되지 않음**.

  `gps_processor.cpp`의 실제 호출 경로:
  - 랩 기록: `startRecordingLap()`, `addPointToRecording()` → `lap_storage.cpp` 직접
  - 레퍼런스 랩: `setReferenceLap()`, `calculateDelta()` → `delta_calculator.cpp` 직접
  - `lap_manager`의 `startLap()`, `completeLap()`, `loadRefLapFromStorage()` 등은 **어디서도 호출되지 않음**

  추가 발견된 문제들:
  - `loadRefLapFromTrackData()`, `loadReferenceLapFromPriorSession()` — Everland 하드코딩 (`getEverlandLapBoundaries()`, `getEverlandReferencePoints()`) 레거시 참조
  - `loadRefLapFromStorage()` (L247-252) — TODO만 있고 구현 없음, `return false`
  - `saveAsReferenceLap()` (L381-399) — TODO 주석만 있음
  - `printf` 사용 (ESP_LOG 미사용) — 빌드 환경 불일치
  - `s_initialized` 변수 설정 후 미사용

  **선택지 B(삭제) 권장**: 현재 기능을 `lap_storage` + `delta_calculator`가 완전히 대체 중. 통합(A)을 시도하면 이미 작동하는 두 모듈을 흔들어야 함.

- 근거 파일:
  - `components/timing/lap_manager.cpp` (전체)
  - `components/timing/lap_manager.h` (전체)
- 기대 효과: 중복 책임 제거, 코드 탐색 비용 감소. 삭제 후 빌드 확인으로 완료.

### 6. 델타 계산 엣지케이스 보정
- 우선순위: Medium → **[코드 검토 후] Low로 하향 권장**
- 문제: `getReferenceTimeAtDistance()` 결과 0ms를 실패로 간주하는 로직이 랩 시작점과 충돌 가능.
- **[코드 검토 후 업데이트]** 진단 자체는 정확하나, **해당 함수는 현재 코드베이스에서 호출되지 않는 데드코드**.

  **이슈 위치** (`delta_calculator.cpp` L311-317):
  ```cpp
  unsigned long refTimeMs = getReferenceTimeAtDistance(trackDistanceM);
  if (refTimeMs == 0) { return result; }  // ← 0=실패 간주 → 랩 시작점에서 오탐
  result.deltaSeconds = (float)(currentLapTimeMs - refTimeMs) / 1000.0f;
  //                           ↑ unsigned 차감: car 앞설 때 언더플로우 → 엉뚱한 큰 양수
  ```

  **실제 사용 경로**: `gps_processor.cpp`는 `calculateDelta()` (L402)만 사용하며, `calculateDeltaAtDistance()`는 헤더에 선언만 있고 **어디서도 호출되지 않음** (grep 확인).
  실제 사용 중인 `calculateDelta()`에서는 `float` 연산으로 delta를 계산하므로 unsigned 언더플로우 문제 없음.

  **결론**: 버그 자체는 실재하나 현재 배포 코드에 영향 없음. `calculateDeltaAtDistance()`를 실제로 사용할 시점에 함께 수정하면 됨. 지금 당장 Medium 우선순위로 다룰 필요 없음.

- 근거 파일:
  - `components/timing/delta_calculator.cpp` (L300-321: `calculateDeltaAtDistance` — 현재 미사용)
- 기대 효과: 향후 섹터 타이밍에서 distance-based delta를 사용할 때 정확도 확보

### 7. 대형 파일 분할 (`waveshare_display.cpp`)
- 우선순위: Low (**[코드 검토 후 확인] 타당, 분할 시 주의사항 추가**)
- 문제: LCD 드라이버, LVGL 렌더링, UI 업데이트, 전원 제어가 한 파일에 과집중.
- **[코드 검토 후 업데이트]** 방향은 옳으나, 분할 실행 시 몇 가지 주의 필요.
  - `waveshare_display.cpp`는 항목 4의 shim 헤더(`components/waveshare_display.h` → `main/waveshare_display.h`)를 통해 컴포넌트에서 참조되는 구조. 분할 시 외부 노출 API 시그니처를 건드리면 shim 헤더와의 관계도 함께 정리 필요.
  - `extern void updateDisplayData()` 같이 `gps_processor.cpp`에서 extern으로 직접 참조하는 함수들이 어느 파일로 이동하는지 명확히 해야 함.
  - 분할 우선순위: `display_hal` (패널/터치/백라이트) 먼저 분리가 가장 독립적이고 안전.

- 근거 파일:
  - `main/waveshare_display.cpp`
- 기대 효과: 변경 영향 범위 축소, 디버깅/리뷰/테스트 난이도 감소

## 3. 현재 잘된 부분
1. 설정 상수가 `config.h`로 중앙화되어 조정 포인트가 명확함.
2. PageManager 기반 페이지/서브시스템 전환 구조가 비교적 명료함.
3. 델타 계산의 윈도우 검색 + fallback 전략은 실용적임.
4. 스토리지 마이그레이션/폴백(legacy 대응) 방향이 현실적임.

## 4. 실행 순서 제안
1. 안전성 핫픽스
   - 항목 1, 2, 3 처리
2. 경계/책임 정리
   - 항목 4, 5 처리
3. 계산 안정화
   - 항목 6 처리 + 테스트
4. 구조 개선
   - 항목 7 단계적 분할

## 5. 완료 기준 (Definition of Done)
1. PRE_TRACK 자동 세션 진입이 재현 가능한 시나리오에서 안정적으로 동작한다.
2. OTA는 인증 없는 업로드가 차단되고, 실패 원인이 로그로 구분된다.
3. 손상된 lap 파일 입력 시 과할당 없이 안전하게 실패한다.
4. 컴포넌트 include 경계가 정리되어 `main` 의존이 축소된다.
5. 델타 계산 경계 케이스(시작점/종료점/빈 레퍼런스) 테스트가 통과한다.
