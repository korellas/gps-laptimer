# GPS Lap Timer 리팩터링 검토 문서

작성일: 2026-02-18
2차 리뷰: 2026-02-20
대상: `main/`, `components/` 전반
목적: 리팩터링 우선순위 확정 및 실행 기준 정리

## 1. 한눈에 요약

**[코드 검토 후 우선순위 재조정]**

| 항목 | 원래 우선순위 | 검토 후 | 비고 |
|---|---|---|---|
| 1. PRE_TRACK 감지 | Critical | Critical | 구조적 항상 실패 → 이미 수정됨 |
| 2. OTA 보안 | High | Medium | 개인 장치 특성, WPA2만으로 대부분 해소 |
| 3. pointCount 검증 | High | High | 코드 확인, 패치 범위 최소 |
| 4. 모듈 경계 | Medium | Medium | shim 헤더 6개 확인됨 |
| 5. lap_manager | Medium | Medium(삭제) | 전혀 미사용, 즉시 삭제 권장 |
| 6. delta 엣지케이스 | Medium | Low | 해당 함수 데드코드 확인 |
| 7. waveshare 분할 | Low | Low | 방향 타당, 분할 주의사항 추가 |

1. 먼저 고쳐야 할 항목은 기능 정확성(PRE_TRACK — 수정 완료), 저장 파일 로드 안전성(pointCount)입니다.
2. 그 다음은 미사용 코드 삭제(`lap_manager`), OTA WPA2 패스워드 추가입니다.
3. 마지막으로 모듈 경계 정리, 대형 파일 분리를 단계적으로 진행합니다.
4. `delta_calculator`의 엣지케이스는 해당 함수를 실제 사용할 시점에 같이 수정합니다.

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

> **[2차 리뷰 코멘트 — 2026-02-20]**
> 수정 확인 완료. 현재 코드(gps_processor.cpp L62-414)에서 `s_preTrackPrev[BUILTIN_TRACK_COUNT]` 배열로 트랙별 이전 포인트를 독립 관리하고, `segmentsIntersect()`/bearing 체크를 인라인으로 수행하며, `setFinishLineFromDefinition()`은 교차 확인 성공 후 1회만 호출하는 구조로 완전히 재설계됨. L337-343에 원래 버그와 수정 근거가 주석으로 문서화되어 있음. **이 항목은 Close.**

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

> **[2차 리뷰 코멘트 — 2026-02-20]**
> WiFi `WIFI_AUTH_OPEN` (L943), BLE `min_key_size=0` (L345/355/365) 모두 현재 코드에서 확인됨. 분석 정확. 우선순위 Medium 동의 — 개인 장치 + 수동 활성화 특성상 WPA2 패스워드 1줄이 가성비 최고.

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

> **[2차 리뷰 코멘트 — 2026-02-20]**
> 미수정 확인. `loadLap()` L250, `loadBestLapFromFile()` L304 모두 `resize(header.pointCount)` 전에 상한 검증 없음. `MAX_POINTS_PER_LAP = 12000`이 `lap_storage.h` L21에 이미 정의되어 있고 `addPointToRecording()`에서는 사용 중이나, 로드 경로에서는 미사용. `LapHeader.pointCount`가 `uint16_t`이므로 최대 65535, `sizeof(StoredPoint)=15` 기준 약 983KB — ESP32-S3 힙 초과 가능. 즉시 패치 권장 동의.
> 추가 제안: `loadLap()`과 `loadBestLapFromFile()`이 95% 동일한 로직이므로 공통 헬퍼 `readLapFromFile(FILE*, StorableLap&)`로 추출하면 검증 로직 1곳만 관리 가능.

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

> **[2차 리뷰 코멘트 — 2026-02-20]**
> shim 헤더 6개, extern 패턴 모두 확인. 문서에 누락된 추가 발견: `simulation.cpp` (L44)에서도 동일한 `extern void updateDisplayData(...)` 선언 사용. 즉, extern 직접 참조는 `gps_processor.cpp` + `simulation.cpp` 2곳. `serial_commands.h` shim은 현재 어디서도 include되지 않는 미사용 shim — 삭제 대상.

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

> **[2차 리뷰 코멘트 — 2026-02-20]**
> "전혀 사용되지 않음"은 **부분적으로 오류**. `simulation.cpp` (L26)에서 `lap_manager.h`를 include하며, `loadReferenceLapFromPriorSession()` (L136)과 `loadRefLapFromTrackData()` (L142, L210)을 호출 중. 약 38개 공개 함수 중 실제 외부 호출은 이 2개뿐이고 나머지 ~36개는 진짜 데드코드.
> **수정 권장**: "전혀 미사용"이 아니라 "simulation.cpp에서 2개 함수만 사용, 나머지 ~36개 미사용". 삭제(B) 대신 **단계적 축소**가 더 안전: (1) 미사용 36개 함수 먼저 제거, (2) simulation.cpp의 레퍼런스 랩 로딩 로직을 `lap_storage`로 이전, (3) 그 후 `lap_manager` 전체 삭제. 또한 `gps_processor.h` L95의 stale 주석 (`// Note: isLapInProgress() is defined in timing/lap_manager.h`) 도 같이 정리.

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

> **[2차 리뷰 코멘트 — 2026-02-20]**
> 데드코드 확인. 라인 번호 소폭 수정: 실제 `calculateDeltaAtDistance`는 L310-331 (문서에는 L300-321로 기재). `calculateDelta()`의 float 연산 안전성도 확인 — L273-274에서 `curTimeSec`/`refTimeSec` 모두 float 변환 후 차감. Low 하향 동의.
> 추가 발견: `initDeltaCalculator()` (L33-39)도 데드코드 — 코드베이스 전체에서 호출 없음. `s_initialized` 플래그 설정 후 아무 조건문에서도 사용 안 됨. 향후 정리 시 함께 삭제 권장.

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

> **[2차 리뷰 코멘트 — 2026-02-20]**
> 2,434줄 확인. 20개 이상 섹션 (HAL/터치/LVGL/UI 생성/업데이트/알림/시작화면/GPS 상태/IMU 상태 등) — 문서의 "과집중" 진단보다 실제로 더 심각.
> **사실 관계 수정**: 문서가 `updateDisplayData()`를 waveshare에 있는 것처럼 기술했으나, 실제로는 `main/main.cpp` L386에 정의됨. waveshare_display.cpp에 있는 건 `updateLapData()`, `updateGpsData()`, `updateTimeData()` 등 개별 위젯 업데이트 함수들. 분할 시 이 함수들의 위치만 관리하면 되고, `updateDisplayData`는 main.cpp에 그대로 유지 가능.

## 2-B. 추가 발견 항목 (2차 리뷰, 2026-02-20)

코드베이스 전체를 재검토하여 기존 문서에서 다루지 않은 이슈를 발견했습니다.

### 8. [Critical] gApp 센서 퓨전 필드 레이스 컨디션
- 우선순위: Critical
- 문제: `imuTask` (core 1, 100Hz)가 `gApp.imuData`, `gApp.fusedSpeedKmh`, `gApp.fusionActive` 등을 읽고 씀. 동시에 LVGL 태스크 (core 0)가 `waveshare_display.cpp` L2345-2352에서 같은 필드를 읽음 — **크로스 코어 접근에 동기화 없음**.
- `ImuData`는 13개 필드로 구성된 구조체라 중간에 컨텍스트 스위치 시 부분적으로 갱신된 값을 읽을 수 있음.
- 근거 파일:
  - `main/main.cpp` (L314: imuTask, L361: xTaskCreatePinnedToCore)
  - `components/modes/gps_processor.cpp` (L266, L422)
  - `main/waveshare_display.cpp` (L2345-2352: LVGL에서 imuData 읽기)
- 제안: `SemaphoreHandle_t` 뮤텍스로 센서 퓨전 필드 보호, 또는 큐 기반으로 imuTask→메인 태스크 단방향 전달

### 9. [High] printf()가 delta_calculator 핫 경로에서 사용
- 우선순위: High
- 문제: `delta_calculator.cpp`에 6곳 (L38, 52, 66, 87, 123, 247)에서 `printf()` 사용. 특히 L123의 `[DeltaCalc] Track distance:` 출력은 GPS 업데이트마다 (10Hz) 실행됨.
- `printf()`는 `_REENT` 뮤텍스 획득 + UART 출력 → ESP_LOGx와 달리 로그 레벨로 억제 불가.
- 근거 파일: `components/timing/delta_calculator.cpp`
- 제안: 모든 `printf()`를 `ESP_LOGx(TAG, ...)` 로 교체

### 10. [High] sendCfgValset() 버퍼 오버플로우 체크 위치 오류
- 우선순위: High
- 문제: `ublox_gps.cpp` L525에서 payload 경계 체크가 아이템 기록 **후**에 수행됨. 마지막 유효 아이템 시 8바이트 기록으로 `payload[128]` 끝을 1바이트 초과 가능.
- 현재 호출 사이트에서는 최대 4개 아이템 (36바이트)이라 트리거되지 않지만, 향후 아이템 추가 시 잠재적 버퍼 오버플로우.
- 근거 파일: `main/ublox_gps.cpp` (L502-532)
- 제안: 경계 체크를 아이템 기록 **전**으로 이동: `if (pos + 4 + items[i].size > (int)sizeof(payload) - 2)`

### 11. [High] getNextSessionId() — uint16_t 오버플로우
- 우선순위: High
- 문제: `lap_storage.cpp` L569에서 파일명의 세션 ID를 `atoi()`로 읽어 `uint16_t maxSession`에 대입. 65535를 초과하는 값 (손상 SD 카드, `s99999_l001.bin` 등)에서 자동 절사 → 세션 ID 0 또는 1로 기존 파일 덮어쓰기.
- 근거 파일: `main/lap_storage.cpp` (L558-578)
- 제안: `sid`를 `[1, 65534]` 범위로 클램핑

### 12. [High] IMU 태스크 핸들 미보관
- 우선순위: High
- 문제: `main.cpp` L361에서 `xTaskCreatePinnedToCore(imuTask, ..., NULL, 1)`로 생성 시 `TaskHandle_t`를 NULL로 버림. 모드 전환이나 리셋 시 IMU 태스크를 정지/일시중지할 수 없음.
- 근거 파일: `main/main.cpp` (L361)
- 제안: `static TaskHandle_t s_imuTaskHandle;`로 핸들 보관

### 13. [High] wifi_portal cJSON_PrintUnformatted NULL 미검사
- 우선순위: High
- 문제: `saveSettings()` (wifi_portal.cpp L1027)에서 `cJSON_PrintUnformatted` 반환값이 NULL일 때 (힙 부족) `fwrite(json, 1, strlen(json), f)`에서 `strlen(NULL)` → UB/크래시. `handleGetSettings()` (L197)에서도 `httpd_resp_send(req, NULL, HTTPD_RESP_USE_STRLEN)` 내부에서 NULL 역참조.
- `axis_calibrator.cpp` L379는 정상적으로 NULL 체크함.
- 근거 파일: `components/wifi_portal/wifi_portal.cpp` (L197, L1027-1031)
- 제안: `if (!json) { cJSON_Delete(root); return; }` 추가

### 14. [High] static otaBuf/fileBuf — 비재진입 + 상시 RAM 점유
- 우선순위: High
- 문제: `wifi_portal.cpp`에서 `static uint8_t otaBuf[4096]` (L324), `static char fileBuf[512]` (L689) 선언. OTA가 미실행 중에도 내부 RAM 4.5KB 상시 점유. `max_open_sockets=2`에서 동시 요청 시 동일 버퍼 공유로 데이터 손상 가능.
- 근거 파일: `components/wifi_portal/wifi_portal.cpp` (L324, L689)
- 제안: `malloc()`/`free()`로 교체하여 필요 시에만 할당

### 15. [High] handlePostSettings() HTTP 바디 절삭
- 우선순위: High
- 문제: `wifi_portal.cpp` L211에서 `char buf[256]` 선언 후 `httpd_req_recv(req, buf, sizeof(buf)-1)`로 최대 255바이트만 읽음. `content_len` 미검사 → 긴 요청에서 JSON이 잘려 `cJSON_Parse`가 불완전한 데이터를 파싱할 수 있음.
- 근거 파일: `components/wifi_portal/wifi_portal.cpp` (L211-212)
- 제안: `req->content_len >= sizeof(buf)` 시 413 반환, 또는 힙 할당

### 16. [Medium] smoothPoint()에서 initialized 미설정
- 우선순위: Medium
- 문제: `gps_filter.cpp` L111-177에서 `GPSPoint smoothed`를 생성하고 좌표/속도/시간 등을 채우지만 `smoothed.initialized = true`를 설정하지 않음. 이후 `isValid()`가 false 반환 → spike 감지가 비활성화될 수 있음.
- 근거 파일: `components/geo/gps_filter.cpp` (L111-177)
- 제안: `smoothed.initialized = true;` 추가

### 17. [Medium] finishRecordingLap()의 totalTimeMs 부정확
- 우선순위: Medium
- 문제: `lap_storage.cpp` L489에서 `outLap.totalTimeMs = outLap.points.back().lapTimeMs`로 마지막 기록 포인트의 시간을 사용. 실제 교차 시점과 최대 1 GPS 프레임 (100ms) 차이 가능. 표시되는 랩 타임은 정확한 `lapTimeMs`를 사용하지만 저장 파일의 `totalTimeMs`가 다름.
- 근거 파일: `main/lap_storage.cpp` (L489)
- 제안: `onLapComplete()`에서 `completedLap.totalTimeMs = lapTimeMs`로 교차 시점 기준으로 덮어쓰기

### 18. [Medium] ets_delay_us(200) × 12 = 2.4ms CPU 블로킹
- 우선순위: Medium
- 문제: `main.cpp` L223-225에서 배터리 ADC 읽기 시 `ets_delay_us(200)` 12회 반복 = 2.4ms 비지웨이트. FreeRTOS 스케줄러 차단.
- 60초 간격 실행이라 실질 영향 작지만, GPS 10Hz 프레임 처리 도중 블로킹 발생 가능.
- 근거 파일: `main/main.cpp` (L223-225)
- 제안: `vTaskDelay(1)`로 교체하거나 저우선순위 백그라운드 태스크에서 측정

### 19. [Medium] rename() 반환값 미확인 (lap 마이그레이션)
- 우선순위: Medium
- 문제: `lap_storage.cpp` L136-137에서 `rename(oldPath, newPath)` 반환값 무시. SPIFFS↔SD 크로스 파일시스템 rename은 POSIX에서 항상 실패 → 마이그레이션 실패가 무시됨.
- 근거 파일: `main/lap_storage.cpp` (L136-137)
- 제안: 반환값 확인, 크로스 디바이스 시 read-copy-delete 사용

### 20. [Medium] fread() 반환값 미확인 (축 캘리브레이션 로드)
- 우선순위: Medium
- 문제: `axis_calibrator.cpp` L306-308에서 `fread(buf, 1, size, f)` 반환값 미검사. SPIFFS 손상 시 불완전 데이터로 `cJSON_Parse` 진행 가능.
- 근거 파일: `components/sensor_fusion/axis_calibrator.cpp` (L306-308)
- 제안: `fread` 반환값 == `size` 확인, 아니면 return false

### 21. [Low] I2C 디바이스 핸들 누수 (IMU WHO_AM_I 실패)
- 우선순위: Low
- 문제: `qmi8658c.cpp` L136-141에서 `i2c_master_bus_add_device()` 성공 후 WHO_AM_I 미스매치 시 `s_imu_dev = nullptr`만 하고 `i2c_master_bus_rm_device()` 미호출 → I2C 디바이스 핸들 누수. 부트 시 1회만 호출되므로 실질 영향 작음.
- 근거 파일: `components/imu/qmi8658c.cpp` (L136-141)
- 제안: 실패 경로에서 `i2c_master_bus_rm_device(s_imu_dev)` 호출 후 nullptr 설정

### 22. [Low] getBestLapPathForTrack() 정적 버퍼 반환
- 우선순위: Low
- 문제: `lap_storage.cpp` L39-49에서 `static char buf[128]`을 반환. 현재 단일 스레드 호출이라 문제없으나, 여러 호출자가 동시에 포인터를 보유하면 덮어쓰기 발생 가능.
- 근거 파일: `main/lap_storage.cpp` (L39-49)
- 제안: 호출자 제공 버퍼 방식 또는 `std::string` 반환으로 변경

## 3. 현재 잘된 부분
1. 설정 상수가 `config.h`로 중앙화되어 조정 포인트가 명확함.
2. PageManager 기반 페이지/서브시스템 전환 구조가 비교적 명료함.
3. 델타 계산의 윈도우 검색 + fallback 전략은 실용적임.
4. 스토리지 마이그레이션/폴백(legacy 대응) 방향이 현실적임.

## 4. 실행 순서 제안 (2차 리뷰 반영, 2026-02-20)

### Phase 1: 즉시 패치 (범위 최소, 리스크 낮음)
- 항목 3: `loadBestLapFromFile` / `loadLap`에 pointCount 상한 검증 + 공통 헬퍼 추출
- 항목 9: `delta_calculator.cpp`의 `printf()` → `ESP_LOGx()` 교체 (6곳)
- 항목 13: `wifi_portal.cpp`의 `cJSON_PrintUnformatted` NULL 체크 추가 (2곳)
- 항목 16: `gps_filter.cpp` `smoothPoint()`에 `initialized = true` 추가

### Phase 2: 단기 안정성 (1-2일)
- 항목 8: **gApp 레이스 컨디션** — 센서 퓨전 필드에 뮤텍스 적용 (Critical)
- 항목 12: IMU 태스크 핸들 저장
- 항목 10: `sendCfgValset()` 경계 체크 위치 수정
- 항목 2: WiFi AP WPA2 패스워드 추가
- 항목 14: static otaBuf/fileBuf를 동적 할당으로 교체

### Phase 3: 기능 개선 (3-5일)
- 항목 5: `lap_manager` 단계적 축소 (미사용 36개 함수 제거 → simulation.cpp 의존 이전 → 삭제)
- 항목 11: `getNextSessionId()` 오버플로우 방지
- 항목 17: `finishRecordingLap()`의 totalTimeMs 정확도 개선
- 항목 19-20: 파일 I/O 에러 처리 보강 (rename, fread)

### Phase 4: 중기 리팩터링
- 항목 4: shim 헤더 및 extern 직접 참조 정리
- 항목 7: `waveshare_display.cpp` 분할 (`display_hal` 먼저)

### Phase 5: 사용 시점에 처리
- 항목 6: `calculateDeltaAtDistance` 실제 사용 시 함께 수정
- 항목 15: HTTP POST 바디 크기 검증 (WiFi 설정 경로 확장 시)

## 5. 완료 기준 (Definition of Done)
1. ~~PRE_TRACK 자동 세션 진입이 재현 가능한 시나리오에서 안정적으로 동작한다.~~ → **Done (v1.2.0)**
2. OTA는 인증 없는 업로드가 차단되고, 실패 원인이 로그로 구분된다.
3. 손상된 lap 파일 입력 시 과할당 없이 안전하게 실패한다.
4. 컴포넌트 include 경계가 정리되어 `main` 의존이 축소된다.
5. 델타 계산 경계 케이스(시작점/종료점/빈 레퍼런스) 테스트가 통과한다.
6. **(신규)** `gApp` 센서 퓨전 필드가 뮤텍스로 보호되어 크로스 코어 레이스 컨디션이 없다.
7. **(신규)** 모든 파일 I/O 경로에서 에러 핸들링이 완비되어 SPIFFS/SD 손상 시 안전하게 실패한다.
