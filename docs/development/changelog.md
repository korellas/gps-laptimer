# 변경 로그

GPS 랩 타이머 프로젝트의 모든 주목할 만한 변경사항.

형식은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/)를 기반으로 합니다.

---

## [1.1.0] - 2026-02-19

### 추가됨
- **페이지 시스템** — PageManager + Page 인터페이스 기반 10개 페이지 (ModeSelect, PhonePlate, WaitGPS, BleOTA, Settings, GPSStatus, PreTrack, Laptimer, Emulation, Transition)
- **센서 퓨전** — Wahba's problem (SVD) 축 캘리브레이터 + 1D Speed Kalman 필터
  - 캘리브레이션: GPS velNED 차분 vs IMU 가속도 → 회전 행렬 R (sensor→NED)
  - Speed KF: predict@100Hz (IMU fwdAccel), update@10Hz (GPS 속도)
  - 저장: `/spiffs/config/imu_fusion.json`, 부팅 시 재사용
  - GPS 단절(터널) 시에만 융합 속도 활용, 평상시 raw GPS 속도 사용
- **결승선 통과로 트랙 식별** — PRE_TRACK 상태에서 60+ km/h 시 모든 트랙 피니시라인 동시 체크 → 트랙 자동 식별
- **트랙별 베스트랩 저장** — `best_{trackId}_{layoutId}.bin`, LapHeader에 trackIndex/layoutIndex 포함 (v2 포맷)
- **인제스피디움 레이아웃 추가** — full, south, north 3개 레이아웃
- **PreTrack 페이지** — 센서 퓨전 캘리브레이션 진행 상태 표시

### 변경됨
- **세션 상태머신 단순화** — NEAR_TRACK 상태 제거, PRE_TRACK → SESSION_ACTIVE 직접 전환
- **기본 GPS 모드** — 부팅 시 `GPSMode::GPS_HARDWARE` (기존: SIMULATION)
- **GPS 부팅 즉시 활성화** — 어느 페이지에서나 GPS 켜져 있음, WiFi는 SETTINGS 페이지에서만 시작

---

## [1.0.0] - 2026-02-17

### 추가됨
- **SD 카드 로깅** — 세션 CSV + 이벤트 로그 (`sd_logger` 컴포넌트)
- **WiFi 파일 브라우저** — 설정 페이지에서 SoftAP 시작, HTTP 파일 브라우저 및 OTA UI

### 수정됨
- **인제 피니시라인 좌표** — 정밀 GPS 측정값으로 업데이트

---

## [0.9.11] - 2026-02-16

### 추가됨
- **인제스피디움 초기 추가** — full 레이아웃 피니시라인 및 기본 섹터
- **BLE OTA** — `ble_ota` 컴포넌트, BleOTA 페이지
- **전력 관리** — PM DFS (80-160MHz) + Tickless Idle
- **프로덕션 빌드 설정** — `sdkconfig.defaults.prod` (WARN 로그, silent assertions)

---

## [Unreleased]

_아래는 릴리스 태그 이전에 개발된 항목들 (v0.4.0 ~ v0.9.10 구간)_

### 추가됨
- SSOT 문서화 구조 (reference/, planning/, development/, guides/)
- 문서화 인덱스 (docs/README.md)
- 포괄적인 참조 문서 (아키텍처, 하드웨어, UI, 데이터 구조, 빌드, 트랙)
- 계획 구조 (로드맵, 기능 백로그)
- **전용 LVGL Task** - Core 0에서 독립 실행 (Waveshare 예제 방식)
- **LVGL indev 터치** - 11-byte I2C 프로토콜로 좌표 파싱 지원
- **Status Bar** - 상단에 배터리/GPS/시계 상태바 추가
  - 배터리 아이콘: 라운드 사각형 (white 외곽, green/red fill), 충전량 비례
  - GPS 위성 상태: 위성 아이콘 + 10개 신호 바 (활성=white, 비활성=outline), >10 위성 시 "+"
  - 시계: 12시간제 KST (`h:mm AM/PM`), GPS UTC 시간에서 동기화
- **GPS 시간 파싱** - UBX NAV-PVT에서 UTC 시간 추출 (year/month/day/hour/min/sec)
- **시스템 시계 동기화** - GPS 시간으로 `settimeofday()` 호출 (최초 1회)
- **시작 화면** (Phase 2.4) - 부팅 시퀀스 UI
  - 로고/프로젝트명 "GPS LAPTIMER" + 버전 표시 (Share Tech Mono 56pt)
  - 모드 선택: 시뮬레이션/GPS 모드 (스와이프/탭으로 전환)
  - GPS fix 대기 화면: 위성 수, 상태 텍스트, 애니메이션
  - 전화번호판 입력 (선택)
  - StartupState 상태 머신: INIT → MODE_SELECT → PHONE_PLATE → WAIT_SIM/WAIT_GPS → TRANSITION → DONE
- **배터리 모니터링** (Phase 2.5) - 전용 FreeRTOS 태스크 (Core 1, Priority 1)
  - IO4 ADC + 1:3 분압기, ADC_ATTEN_DB_12, 12-bit
  - 13회 오버샘플링 + 상하위 25% 트리밍 + 중간값 평균
  - 21-entry LiPo 방전 곡선 LUT + 선형 보간으로 SoC% 계산
  - EWMA 방전율 추적 (alpha=0.05, 5분 시간 상수) → 남은 시간 예측 (`gApp.batteryMinLeft`)
  - 배터리 아이콘: SoC 비례 fill (green >20%, red ≤20%)
- **GPIO16 전원 OFF** - PWR 버튼 2초 롱프레스 시 시스템 전원 차단 (IO Expander PIN6)
- **WiFi 캡티브 포털** - ESP32 SoftAP 모드
  - SSID "LAPTIMER" (인증 없음)
  - DNS 리다이렉트 + HTTP 서버 (임베디드 HTML)
  - 설정 변경 웹 인터페이스
- **코드 리뷰 보고서** (2026-02-16) - 전체 코드베이스 성능/전력/품질 분석
- **GPS 모듈 설정** (2026-02-16) - 부팅 시 9600→115200 baud 전환, CFG-RATE 10Hz, Automotive 모드
- **GPS 필터/DR 연결** - `processRealGPS()`에 `filterGPSPoint()` + Dead Reckoning 통합
- **GPS newData 판정** - `s_newFrameParsed` 플래그로 새 프레임 감지 (iTOW 비교 불필요)
- **GPS 모듈 전원 관리** - `enableGPSModule()`/`disableGPSModule()` (GPIO0 ENABLE)
- **전력 관리** (2026-02-16) - PM DFS (80-160MHz) + Tickless Idle + WiFi 라이프사이클
- **프로덕션 빌드** - `sdkconfig.defaults.prod` (WARN 로그, silent assertions)
- **lvgl_mutex 최적화** - `updateLapData()` 연산/LVGL 분리 (mutex 보유 시간 단축)

### 변경됨
- SSOT 원칙에 따라 문서화 재구성
- CLAUDE.md를 링크 기반으로 업데이트 (중복 없음)
- README.md를 최소 소개로 업데이트
- **LVGL Tick**: 16ms → 5ms (Waveshare 예제와 동일)
- **LCD Reset 시퀀스**: 부팅 직후 RST=LOW 고정 → 전원/SPI 설정 → RST=HIGH + 120ms 대기
- **Init commands**: Waveshare 예제와 동일하게 `0x11`(100ms) + `0x29`(100ms)만 사용
- **UI 레이아웃**: 기존 요소 (LAP, 섹터, 베스트랩, 델타) +16px 하향 이동 (status bar 공간 확보)
- **vTaskDelay(1) → vTaskDelay(5)** - 메인 루프 폴링 80% 감소
- **터치 I2C 타임아웃** - 50ms → 20ms
- **LVGL 플러시 타임아웃** - 200ms → 50ms (3개소)
- **시간 업데이트** - 30Hz → 1Hz (매 프레임 시스템 콜 제거)
- **DEBUG_OUTPUT** - `printf()` → `ESP_LOGD()` 통일 (sector_timing, dead_reckoning, gps_filter, simulation)
- **매직넘버** - `5000` → `SECTOR_DELTA_DISPLAY_MS`, `2.0f` → `SECTOR_PROGRESS_MAX`
- **섹터 좌표** - `sector_timing.cpp` 하드코딩 제거 → `builtin_tracks.h` SSOT
- **WiFi** - `initWifiPortal()` → `initWifiPortal()`+`startWifiPortal()`+`stopWifiPortal()` 라이프사이클 분리
- **FinishLine** - `FinishLine` → `FinishLineDefinition` 리네임 (track_types.h)

### 수정됨
- **콜드부트 실패 해결** - 전원 OFF→ON 시 디스플레이 초기화 실패하던 문제
  - 근본 원인: `esp_lcd_panel_swap_xy()`, `esp_lcd_panel_mirror()`, `esp_lcd_panel_disp_on_off()`가 QSPI 모드에서 AXS15231B를 손상시킴
  - 수정: 해당 호출 제거, RST 핀을 부팅 즉시 LOW로 고정, 예제와 동일한 init 시퀀스 적용
- **GPS 위성 수 하드코딩 수정** - GPS 모드에서 `gframe.sats`가 10으로 고정되어 있던 문제 → `getUBloxData().satellites` 사용

### 수정됨 (2026-02-16 리팩토링)
- **GPS newData 버그** - `lastData.valid` → `s_newFrameParsed` 플래그 (중복 처리 방지)
- **GPS 첫 베스트랩** - `onLapComplete()`에서 `setReferenceLap()` 즉시 반영 (기존: SPIFFS만 저장)
- **httpd_uri 경고** - wifi_portal.cpp의 9개 httpd_uri_t 필드 초기화 추가
- **dns_server.h 경고** - DNS_SERVER_CONFIG_SINGLE 매크로에 `.ip = {}` 추가

### 제거됨
- 흰색 테스트 패턴 (init 후 LCD 검증용, 불필요)
- `handleTouch()` 함수 (LVGL indev로 대체)
- `esp_lcd_panel_swap_xy/mirror/disp_on_off` 호출 (콜드부트 깨짐 원인)
- 배터리 % 텍스트 표시 (`lbl_battery`) → 배터리 아이콘으로 대체
- **`sector.cpp/h`** (495줄) — `sector_timing.cpp` 거리 기반 방식으로 대체
- **미구현 함수 선언** — `gps_processor.h` 6개, `finish_line.h` 2개
- **`initSectorDetection()`** 호출 — simulation.cpp에서 제거

---

## [0.3.0] - 2026-02-08

### 추가됨
- **섹터 타이밍** - 에버랜드 트랙용 3섹터 타이밍
  - 거리 기반 섹터 감지
  - 섹터별 델타 추적
  - 누적 델타 접근법 (모든 섹터 델타가 총 델타로 합산됨)
  - 디스플레이: 좌측 컬럼에 S1/S2/S3 델타 표시 (흰색 텍스트)
- **Share Tech Mono 폰트** - 섹터 델타 및 랩 오버레이용 사용자 지정 폰트
- **상위 3개 랩 디스플레이** - 최고 3개 랩 타임 표시 (계획, 부분 구현)

### 변경됨
- 섹터 델타용 UI 레이아웃 업데이트
- 섹터 경계 감지: 좌표 기반 → 거리 기반
- 레퍼런스 랩 변경 시 섹터 경계 거리 재계산

### 수정됨
- **시뮬레이션 델타 정확도** - simulation.cpp의 wall clock 재정의 타이밍으로 인한 ~0.05초 오프셋 수정
  - 근본 원인: `calculateDelta()` 전 wall clock 재정의
  - 수정: wall clock 재정의를 `calculateDelta()` 후로 이동
  - 영향: 시뮬레이션 모드만 (실제 GPS는 영향 없음)

### 성능
- 거리 계산: `haversine()` → `fastDistanceMetersPrecomp()` (float 전용, 사전 계산된 cos(lat))
- `getCumulativeAtSector()`: O(n) 루프 → O(1) 직접 계산
- `simLapPoints`: `std::vector` → `static GPSPoint[MAX_REFERENCE_POINTS]` (힙 단편화 방지)
- GPS 필터링: `pow(0.7,i)` → `weight *= 0.7f` (곱셈 누적)
- UART GPS 읽기: 1바이트 → 64바이트 청크

---

## [0.2.0] - 2026-02-07

### 추가됨
- **AppContext 패턴** - 단일 `struct AppContext gApp`로 전역 상태 통합
  - 모든 모듈이 `gApp.fieldName`을 통해 접근
  - 상태 관리 단순화
  - 분산된 전역 변수 제거
- **속도 델타 표시** - 하단 바에 속도 차이 (km/h) 표시
  - 시간 델타 (중앙)와 분리
  - 색상 코딩 바 (녹색/빨간색)
  - 바 가장자리에 떠있는 텍스트

### 변경됨
- 분산된 변수에서 `AppContext`로 전역 상태 리팩토링
- `gApp` 접근 패턴을 사용하도록 모든 모듈 업데이트
- 시간 델타 (중앙) 및 속도 델타 (하단 바) 디스플레이 분리

### 수정됨
- 속도에 대한 델타 계산 보간

### 문서화됨
- 코드 리뷰 및 리팩토링 계획 (docs/development/decisions/001-code-review-2026-02-07.md)
- AppContext 패턴에 대한 아키텍처 문서 업데이트

---

## [0.1.0] - 2026-02-06

### 추가됨
- **초기 릴리스** - 핵심 랩 타이머 기능
- **GPS 하드웨어 모드** - 실제 GPS 지원 (u-blox G10A-F33, UBX NAV-PVT)
  - 10 Hz 업데이트 속도
  - 3D fix 추적
  - 속도 및 방향
- **시뮬레이션 모드** - 에버랜드 트랙 데이터 재생
  - 4개 샘플 랩
  - 20 Hz 재생
  - 시간 기반 보간
- **델타 계산** - 레퍼런스 랩 대비 시간 델타
  - 세그먼트 기반 매칭
  - 거리 투영
  - 신뢰도 점수
  - EMA 스무딩 (alpha=0.15)
- **결승선 감지** - 선 세그먼트 교차
  - 방향 검증 (에버랜드는 315°~45°)
  - 데드 존 (30m)
  - 최소 랩 타임 (30초)
- **디스플레이** - AXS15231B의 LVGL 9.x
  - 640×172 가로 UI
  - 30 Hz 업데이트
  - 소프트웨어 회전 (90°)
  - DMA 플러시 파이프라인
- **저장소** - SPIFFS 랩 데이터 영속성
  - 바이너리 랩 데이터 형식
  - 세션 관리
  - 베스트 랩 추적
- **시리얼 명령** - 전체 CLI
  - `e` - GPS/시뮬레이션 토글
  - `r` - 랩 리셋
  - `f` - 결승선 설정
  - `c` - 결승선 클리어
  - `s` - 상태
  - `l` - 랩 목록
  - `n` - 새 세션
  - `h` - 도움말

### 기술 세부사항
- **빌드 시스템:** ESP-IDF v5.5.2
- **타겟:** ESP32-S3
- **디스플레이 드라이버:** AXS15231B (QSPI)
- **GPS 프로토콜:** UBX 바이너리
- **파일 시스템:** SPIFFS
- **UI 프레임워크:** LVGL 9.x

---

## [0.0.1] - 2026-01-xx (프리릴리스)

### 추가됨
- 초기 프로젝트 설정
- PlatformIO → ESP-IDF 마이그레이션
- 2.8" LCD → 3.49" LCD 마이그레이션
- 기본 GPS 파싱
- 기본 디스플레이 출력

---

## 버전 넘버링

우리는 [Semantic Versioning](https://semver.org/)을 사용합니다:
- **MAJOR** 버전: 호환되지 않는 변경 (데이터 형식, API 손상)
- **MINOR** 버전: 새 기능 (하위 호환)
- **PATCH** 버전: 버그 수정 (하위 호환)

---

## 참조

- **현재 상태:** [status.md](status.md)
- **로드맵:** [../planning/roadmap.md](../planning/roadmap.md)
- **아키텍처:** [../reference/architecture.md](../reference/architecture.md)
