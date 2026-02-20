# GPS Lap Timer 리팩터링 계획 (최종)

확정일: 2026-02-20
리뷰 이력: 1차 작성(02-18) → 2차 코드 검증 → 3차 코멘트 → 4차 합의
대상: `main/`, `components/` 전반

## 1. 요약

| # | 항목 | 우선순위 | Phase | 상태 |
|---|---|---|---|---|
| 1 | PRE_TRACK 트랙 감지 | ~~Critical~~ | — | **Done (v1.2.0)** |
| 2 | OTA 보안 (WiFi/BLE) | Medium | 2 | Open |
| 3 | pointCount 상한 검증 | High | 1 | Open |
| 4 | 모듈 경계 (shim 헤더) | Medium | 4 | Open |
| 5 | lap_manager 정리 | Medium | 3 | Open |
| 6 | delta 엣지케이스 | Low | 5 | Open (데드코드) |
| 7 | waveshare 분할 | Low | 4 | Open |
| 8 | gApp same-core 레이스 | High | 2 | Open |
| 9 | printf → ESP_LOGx | Medium | 1 | Open |
| 10 | sendCfgValset 경계 체크 | Medium | 3 | Open (잠재 결함) |
| 11 | getNextSessionId 오버플로우 | High | 3 | Open |
| 12 | IMU 태스크 핸들 미보관 | High | 2 | Open |
| 13 | cJSON NULL 미검사 | High | 1 | Open |
| 14 | static OTA/file 버퍼 RAM 점유 | Medium | 2 | Open |
| 15 | HTTP POST 바디 절삭 | High | 5 | Open |
| 16 | smoothPoint initialized 미설정 | Medium | 1 | Open |
| 17 | totalTimeMs 부정확 | Medium | 3 | Open |
| 18 | ets_delay_us 블로킹 | Medium | 3 | Open |
| 19 | rename() 반환값 미확인 | Medium | 3 | Open |
| 20 | fread() 반환값 미확인 | Medium | 3 | Open |
| 21 | I2C 핸들 누수 | Low | 3 | Open |
| 22 | 정적 버퍼 반환 패턴 | Low | 5 | Open |
| 23 | SD 탈락 오류 처리 | Medium | 3 | Open |

---

## 2. 항목 상세

### 1. [Done] PRE_TRACK 트랙 감지 로직

v1.2.0에서 수정 완료. `s_preTrackPrev[BUILTIN_TRACK_COUNT]` 배열로 트랙별 이전 포인트 독립 관리, `segmentsIntersect()`/bearing 체크 인라인 수행, `setFinishLineFromDefinition()`은 교차 확인 성공 후 1회만 호출. `gps_processor.cpp` L337-343에 원래 버그와 수정 근거 주석 문서화됨.

---

### 2. [Medium] OTA 보안 — WiFi AP WPA2 패스워드 추가

**문제**: WiFi SoftAP가 `WIFI_AUTH_OPEN` (L943), BLE `min_key_size=0` (L345/355/365). 개인 장치 + 수동 활성화 특성상 Medium.

**권장 조치** (비용 대비 효과 순):
1. WiFi AP에 WPA2 패스워드 추가 — 1줄 변경, 가장 cost-effective
2. BLE pairing 요구 (`BLE_GATT_CHR_F_WRITE_AUTHEN`) — 선택적
3. 세션 토큰/이미지 서명은 과잉

**근거**: `wifi_portal.cpp` L943, `ble_ota.cpp` L345-365

---

### 3. [High] pointCount 상한 검증

**문제**: `loadLap()` L250, `loadBestLapFromFile()` L304에서 `resize(header.pointCount)` 전에 상한 검증 없음. `LapHeader.pointCount`가 `uint16_t`(최대 65535), `sizeof(StoredPoint)=15` 기준 약 983KB — ESP32-S3 힙 초과 가능.

**패치**:
```cpp
if (header.pointCount == 0 || header.pointCount > MAX_POINTS_PER_LAP) {
    fclose(f);
    return false;
}
```
`MAX_POINTS_PER_LAP = 12000`은 `lap_storage.h` L21에 이미 정의됨 (recording 경로에서만 사용 중).

**추가**: `loadLap()`과 `loadBestLapFromFile()`이 95% 동일 로직이므로 공통 헬퍼 `readLapFromFile(FILE*, StorableLap&)` 추출 권장.

**근거**: `lap_storage.cpp` L250, L304

---

### 4. [Medium] 모듈 경계 정리 — shim 헤더 + extern 참조

**문제**: `components/` 내 shim 헤더 6개가 `../main/`을 re-export. extern 직접 참조는 `gps_processor.cpp` L39-42 + `simulation.cpp` L44.

**shim 목록**:
```
components/finish_line.h      → ../main/finish_line.h
components/ublox_gps.h        → ../main/ublox_gps.h
components/protocol.hpp       → ../main/protocol.hpp
components/waveshare_display.h→ ../main/waveshare_display.h
components/lap_storage.h      → ../main/lap_storage.h
components/serial_commands.h  → ../main/serial_commands.h  (미사용 — 삭제 대상)
```

**전략 A 확정**: `main/` 핵심 헤더들을 `components/common/`으로 이동하여 의존성 방향을 근본적으로 정리. shim 헤더 6개와 extern 직접 참조를 모두 제거.

---

### 5. [Medium] lap_manager 단계적 축소

**문제**: ~38개 공개 함수 중 외부 호출은 `simulation.cpp`에서 2개뿐 (`loadReferenceLapFromPriorSession`, `loadRefLapFromTrackData`). 나머지 ~36개는 데드코드. Everland 하드코딩 레거시, TODO 스텁, `printf` 사용 등 품질 문제.

**단계적 축소 계획**:
1. 미사용 ~36개 함수 제거, 빌드 확인
2. `simulation.cpp`의 레퍼런스 랩 로딩을 `lap_storage`로 이전
3. `lap_manager.cpp/.h` 전체 삭제 + `gps_processor.h` L95 stale 주석 정리

**근거**: `lap_manager.cpp/.h`, `simulation.cpp` L26/136/142/210

---

### 6. [Low] delta 엣지케이스 — 데드코드

**문제**: `calculateDeltaAtDistance()` (L310-331)에 unsigned 언더플로우 버그 + `refTimeMs==0` 오탐. 단, 이 함수는 코드베이스 전체에서 **호출 없음**. 실사용 중인 `calculateDelta()`는 float 연산으로 안전.

`initDeltaCalculator()` (L33-39)도 데드코드 — `s_initialized` 플래그 미사용.

**조치**: 실제 사용 시점에 함께 수정. 정리 시 데드코드 삭제 포함.

---

### 7. [Low] waveshare_display.cpp 분할

**문제**: 2,434줄, 20+ 섹션 (HAL/터치/LVGL/UI 생성/업데이트/알림/시작화면/GPS 상태/IMU 상태 등).

**분할 우선순위**: `display_hal` (패널/터치/백라이트) 먼저 분리가 가장 독립적.

**주의**: `updateDisplayData()`는 `main.cpp` L386에 정의됨 (waveshare에 있지 않음). waveshare에 있는 건 `updateLapData()`, `updateGpsData()`, `updateTimeData()` 등 개별 위젯 업데이트 함수.

---

### 8. [High] gApp 센서 퓨전 필드 same-core 레이스

**문제**: `imuTask` (core 1, 100Hz)가 `gApp.imuData` 13개 필드를 쓰고, `main_task` (core 1)가 `PageManager::update()` → `updateImuDisplay()` 경로로 같은 필드를 읽음. **같은 코어(core 1)** 에서 동기화 없이 접근 — FreeRTOS 컨텍스트 스위치 시 partial read 가능.

크로스 코어가 아니므로 캐시 일관성 문제는 없음. `taskENTER_CRITICAL()` 또는 atomic copy로 해결 가능.

**근거**: `main.cpp` L314(write)/L935(main_task), `waveshare_display.cpp` L2340(read)

---

### 9. [Medium] printf → ESP_LOGx (delta_calculator)

**문제**: `delta_calculator.cpp`에 `printf()` 6곳. 전부 1회성 또는 1Hz rate-limited — 10Hz 핫패스가 아님. 코드 품질 일관성 차원의 이슈.

| 위치 | 함수 | 호출 빈도 |
|---|---|---|
| L38 | `initDeltaCalculator()` | 부트 1회 (데드코드) |
| L52, L66 | `setReferenceLap()` | 랩 로드 1회 |
| L87 | `clearReferenceLap()` | 클리어 1회 |
| L123 | `calculateCumulativeDistances()` | 랩 로드 1회 |
| L247 | `calculateDelta()` | 최대 1Hz (실패 시) |

---

### 10. [Medium] sendCfgValset 경계 체크 — 잠재 결함

**문제**: `ublox_gps.cpp` L512-528에서 payload 경계 체크가 아이템 기록 **후** 수행. 현재 최대 payload 24/128 bytes로 트리거 불가. 향후 아이템 추가 시 잠재적 오버플로우.

**조치**: 기록 **전** 경계 체크로 이동 + `size` 유효성 검사(1/2/4만 허용).

---

### 11. [High] getNextSessionId — uint16_t 오버플로우

**문제**: `lap_storage.cpp` L569에서 `atoi()`로 파일명의 세션 ID를 읽어 `uint16_t`에 대입. 65535 초과 시 자동 절사로 기존 파일 덮어쓰기 위험.

**조치**: `sid`를 `[1, 65534]` 범위로 클램핑.

---

### 12. [High] IMU 태스크 핸들 미보관

**문제**: `main.cpp` L361에서 `xTaskCreatePinnedToCore(..., NULL, 1)` — `TaskHandle_t` 버림. 모드 전환/리셋 시 IMU 태스크 관리 불가.

**조치**: `static TaskHandle_t s_imuTaskHandle;`로 핸들 보관.

---

### 13. [High] cJSON_PrintUnformatted NULL 미검사

**문제**: `wifi_portal.cpp`에서 `cJSON_PrintUnformatted` 반환 NULL 시:
- `saveSettings()` L1027: `strlen(NULL)` → UB/크래시
- `handleGetSettings()` L197: `httpd_resp_send(req, NULL, ...)` → NULL 역참조

`axis_calibrator.cpp` L379는 정상적으로 NULL 체크 중.

**조치**: `if (!json) { cJSON_Delete(root); return; }` 추가 (2곳).

---

### 14. [Medium] static OTA/file 버퍼 — 상시 4.5KB RAM 점유

**문제**: `static uint8_t otaBuf[4096]` (L324), `static char fileBuf[512]` (L689). WiFi 비활성 시에도 내부 RAM 상시 점유. ESP-IDF httpd는 단일 태스크 기반이라 동시 접근 데이터 손상은 발생하지 않음.

**조치**: `malloc()`/`free()`로 교체하여 필요 시에만 할당.

---

### 15. [High] handlePostSettings HTTP 바디 절삭

**문제**: `wifi_portal.cpp` L211에서 `char buf[256]`, `httpd_req_recv(req, buf, 255)` — `content_len` 미검사. 긴 요청 시 JSON 절삭.

**조치**: `req->content_len >= sizeof(buf)` 시 413 반환, 또는 힙 할당.

---

### 16. [Medium] smoothPoint() initialized 미설정

**문제**: `gps_filter.cpp` L111-177에서 `GPSPoint smoothed` 생성 시 `initialized = true` 미설정. `isValid()` false 반환으로 spike 감지 비활성화 가능.

**조치**: `smoothed.initialized = true;` 1줄 추가.

---

### 17. [Medium] totalTimeMs 부정확 (최대 100ms 오차)

**문제**: `lap_storage.cpp` L489에서 `outLap.totalTimeMs = outLap.points.back().lapTimeMs`. 마지막 기록 포인트와 실제 교차 시점 간 최대 1 GPS 프레임 차이.

**조치**: `onLapComplete()`에서 `completedLap.totalTimeMs = lapTimeMs`로 교차 시점 기준 덮어쓰기.

---

### 18. [Medium] ets_delay_us 블로킹 (배터리 ADC)

**문제**: `main.cpp` L223-225에서 `ets_delay_us(200)` × 12 = 2.4ms 비지웨이트. 60초 간격이라 실질 영향 작음.

**조치**: `vTaskDelay(1)`로 교체 또는 저우선순위 백그라운드 태스크 분리.

---

### 19. [Medium] rename() 반환값 미확인

**문제**: `lap_storage.cpp` L136-137에서 `rename()` 반환값 무시. SPIFFS↔SD 크로스 파일시스템 rename은 POSIX에서 항상 실패.

**조치**: 반환값 확인 + 크로스 디바이스 시 read-copy-delete.

---

### 20. [Medium] fread() 반환값 미확인 (축 캘리브레이션)

**문제**: `axis_calibrator.cpp` L306-308에서 `fread()` 반환값 미검사. SPIFFS 손상 시 불완전 JSON 파싱.

**조치**: `fread` 반환값 == `size` 확인.

---

### 21. [Low] I2C 핸들 누수

**문제**: `qmi8658c.cpp` L136-141에서 WHO_AM_I 실패 시 `i2c_master_bus_rm_device()` 미호출. 부트 1회만이라 실질 영향 미미.

**조치**: 실패 경로에서 `i2c_master_bus_rm_device()` 호출.

---

### 22. [Low] 정적 버퍼 반환 패턴

**문제**: `lap_storage.cpp` L39-49의 `static char buf[128]` 반환. 현재 단일 스레드라 안전하나 패턴이 취약.

**조치**: `std::string` 반환 또는 호출자 제공 버퍼.

---

### 23. [Medium] SD 탈락 시 오류 처리 + 세션 중 fallback 금지

**문제**: `getLapsDir()`가 매 호출마다 `sdcardIsMounted()` 재평가하여 SD 언마운트 시 SPIFFS로 자동 전환됨. 이로 인해 세션 중간에 저장 경로가 바뀌어 데이터 정합성이 깨질 수 있음 (SD에 구버전, SPIFFS에 신버전 → SD 재마운트 시 구버전 읽힘).

**합의된 정책**:
- SD 탈락(세션 중 언마운트)은 **오류로 처리** — SPIFFS로 자동 fallback 하지 않음
- 부트 시점에 SD 마운트 상태를 확정하고, 세션 중에는 저장 경로를 고정
- SD 탈락 감지 시 사용자에게 경고 표시, 쓰기 실패는 로그로 기록

**조치**: `getLapsDir()`를 부트 시 1회 평가 + 캐시하는 구조로 변경. 세션 중 `sdcardIsMounted()` 변화 감지 시 에러 핸들링.

---

## 3. 현재 잘된 부분

1. 설정 상수가 `config.h`로 중앙화되어 조정 포인트가 명확함
2. PageManager 기반 페이지/서브시스템 전환 구조가 명료함
3. 델타 계산의 윈도우 검색 + fallback 전략이 실용적임
4. 스토리지 마이그레이션/폴백(legacy 대응) 방향이 현실적임
5. PRE_TRACK 수정 시 원래 버그와 수정 근거를 주석으로 남긴 점이 좋음

---

## 4. 실행 계획

### Phase 1: 즉시 패치 (범위 최소, 당일 완료 가능)

| # | 항목 | 파일 | 변경량 |
|---|---|---|---|
| 3 | pointCount 상한 검증 + 공통 헬퍼 | `lap_storage.cpp` | ~20줄 |
| 9 | printf → ESP_LOGx | `delta_calculator.cpp` | 6곳 |
| 13 | cJSON NULL 체크 | `wifi_portal.cpp` | 2곳 |
| 16 | smoothPoint initialized | `gps_filter.cpp` | 1줄 |

### Phase 2: 단기 안정성 (1-2일)

| # | 항목 | 파일 | 변경량 |
|---|---|---|---|
| 8 | gApp same-core 레이스 보호 | `main.cpp` | ~15줄 |
| 12 | IMU 태스크 핸들 저장 | `main.cpp` | 2줄 |
| 2 | WiFi WPA2 패스워드 | `wifi_portal.cpp` | 3줄 |
| 14 | static 버퍼 → 동적 할당 | `wifi_portal.cpp` | ~10줄 |

### Phase 3: 기능 개선 (3-5일)

| # | 항목 | 파일 | 비고 |
|---|---|---|---|
| 5 | lap_manager 단계적 축소 | `lap_manager.*`, `simulation.cpp` | 3단계 |
| 10 | sendCfgValset 경계 체크 | `ublox_gps.cpp` | 잠재 결함 |
| 11 | getNextSessionId 오버플로우 | `lap_storage.cpp` | 클램핑 |
| 17 | totalTimeMs 정확도 | `lap_storage.cpp`, `main.cpp` | 100ms 오차 제거 |
| 18 | ets_delay_us 블로킹 | `main.cpp` | 선택적 |
| 19-20 | 파일 I/O 에러 처리 | `lap_storage.cpp`, `axis_calibrator.cpp` | rename, fread |
| 21 | I2C 핸들 누수 | `qmi8658c.cpp` | 부트 1회 |
| 23 | SD 탈락 오류 처리 | `lap_storage.cpp`, `main.cpp` | fallback 금지, 에러 핸들링 |

### Phase 4: 중기 리팩터링

| # | 항목 | 비고 |
|---|---|---|
| 4 | shim 헤더 + extern 정리 | 전략 A 확정: 헤더를 `components/common/`으로 이동 |
| 7 | waveshare_display 분할 | `display_hal` 먼저 |

### Phase 5: 사용 시점에 처리

| # | 항목 | 비고 |
|---|---|---|
| 6 | calculateDeltaAtDistance 수정 | 데드코드, 사용 시 함께 |
| 15 | HTTP POST 바디 크기 검증 | 설정 경로 확장 시 |
| 22 | 정적 버퍼 반환 패턴 개선 | 멀티스레드 전환 시 |

---

## 5. 완료 기준 (Definition of Done)

1. ~~PRE_TRACK 자동 세션 진입이 안정적으로 동작한다~~ → **Done (v1.2.0)**
2. OTA는 인증 없는 업로드가 차단되고, 실패 원인이 로그로 구분된다
3. 손상된 lap 파일 입력 시 과할당 없이 안전하게 실패한다
4. 컴포넌트 include 경계가 정리되어 `main` 의존이 축소된다
5. 델타 계산 경계 케이스(시작점/종료점/빈 레퍼런스) 테스트가 통과한다
6. `gApp` 센서 퓨전 필드가 same-core 레이스로부터 보호된다
7. 모든 파일 I/O 경로에서 에러 핸들링이 완비되어 SPIFFS/SD 손상 시 안전하게 실패한다
8. SD 탈락 시 오류 처리가 동작하고, 세션 중 SPIFFS fallback 없이 안전하게 실패한다
