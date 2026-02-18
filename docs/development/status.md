# 현재 상태

**최종 업데이트:** 2026-02-19

## 🎯 현재 초점

**완료된 작업:**
- ✅ 문서화 재구성 (SSOT 구조) — 완료
- ✅ 디스플레이 초기화 예제 정렬 + 콜드부트 수정 (2026-02-15)
- ✅ 시작 화면 + GPS fix 대기 (2026-02-15)
- ✅ 배터리 모니터링 + EWMA 방전율 추적 (2026-02-15)
- ✅ WiFi 캡티브 포털 (2026-02-15)
- ✅ 코드 리팩토링 (2026-02-16) — Phase 0~3 완료
  - [code-review-2026-02-16-final.md](code-review-2026-02-16-final.md)
- ✅ 전력 관리 최적화 — PM DFS, Tickless Idle, WiFi 라이프사이클
- ✅ 미사용 모듈 정리 — sector.cpp/h 삭제, 미구현 선언 제거
- ✅ GPS 구현 — Baud 115200, 10Hz, Automotive, 필터/DR 연결

**진행 중:**
- 🔄 문서 동기화 (DOC-01~07)
- 📋 저장소/설정 리팩토링 — [specs/storage-settings-refactor.md](../planning/specs/storage-settings-refactor.md)
  - finish_line.bin 삭제, settings.json 통합, NEAR_TRACK 제거, 트랙 JSON 포맷

---

## ✅ 구현된 기능

### 핵심 기능
- ✅ **GPS 하드웨어 모드** - 실제 GPS 처리 (u-blox G10A-F33, UBX 프로토콜)
- ✅ **시뮬레이션 모드** - 개발용 에버랜드 트랙 데이터 재생
- ✅ **델타 계산** - 레퍼런스 랩 대비 시간 델타 + 속도 델타
- ✅ **섹터 타이밍** - 개별 델타 추적을 가진 3개 섹터
- ✅ **랩 관리** - 기록, 저장, 베스트 랩 추적
- ✅ **시리얼 명령** - 전체 CLI (e/r/f/c/s/l/n/d/h)

### UI/디스플레이
- ✅ **LVGL 9.x 통합** - 640×172 가로 디스플레이
- ✅ **전용 LVGL Task** - Core 0, priority 2, 8KB 스택
- ✅ **LVGL indev 터치** - 11-byte I2C 프로토콜 (CST816S)
- ✅ **Status Bar** - 배터리 아이콘 + GPS 신호 바 + KST 시계
- ✅ **시작 화면** - 모드 선택 (시뮬레이션/GPS) + GPS fix 대기 + 전화번호판 입력
- ✅ **섹터 델타 표시** - 좌측 컬럼 (S1/S2/S3), 흰색 텍스트
- ✅ **시간 델타 표시** - 중앙, 큰 글꼴, 흰색
- ✅ **속도 델타 바** - 하단, 색상 코딩 (녹색/빨간색)
- ✅ **Share Tech Mono 폰트** - 24pt/32pt/56pt 다중 사이즈
- ✅ **색상 코딩 바** - 녹색 (#14AA14) / 빨간색 (#AA1414)

### 배터리 및 전원
- ✅ **배터리 ADC 모니터링** - IO4 ADC + 1:3 분압기, 전용 FreeRTOS 태스크
- ✅ **ADC 오버샘플링** - 13회 읽기, 상하위 25% 트리밍, 중간값 평균
- ✅ **EWMA 방전율 추적** - 5분 시간 상수로 남은 시간 예측
- ✅ **21-entry LiPo LUT** - 선형 보간으로 SoC% 계산
- ✅ **GPIO16 전원 OFF** - PWR 버튼 2초 롱프레스 또는 저배터리 시 자동 종료

### 네트워크
- ✅ **WiFi 캡티브 포털** - SoftAP 모드, HTTP 서버, DNS 리다이렉트

### 데이터 및 저장소
- ✅ **저장소** - 랩 데이터 영속성 (SD 우선, SPIFFS 폴백)
- ✅ **레퍼런스 랩 로딩** - 에버랜드 52.83초 레퍼런스
- ✅ **AppContext 패턴** - 단일 전역 상태 (`gApp`)
- ✅ **누적 거리** - 델타 성능을 위해 사전 계산됨

### 성능 최적화 (2026-02-08)
- ✅ **빠른 거리 계산** - float 전용, 사전 계산된 cos(lat)
- ✅ **정적 시뮬레이션 데이터** - 힙 단편화 방지
- ✅ **벡터 사전 예약** - MAX_REFERENCE_POINTS 용량
- ✅ **UART 청크 읽기** - 64바이트 GPS 읽기
- ✅ **O(1) 섹터 조회** - 경계 거리로부터 직접 계산

---

## 🔧 진행 중

### 코드 리팩토링 분석 (2026-02-16)
- 🔄 **전체 코드 리뷰** - 성능/전력/코드품질 종합 분석
  - 결과: [code-review-2026-02-16.md](code-review-2026-02-16.md)
  - 주요 발견: PM 비활성화, WiFi 상시 켜짐, 데드 코드 1300줄+

### 계획
- 📋 **저장소/설정 리팩토링** — [specs/storage-settings-refactor.md](../planning/specs/storage-settings-refactor.md)
  - finish_line.bin 삭제 (dead path)
  - settings.json 통합 확장 (deltaRef, customFinishLine)
  - NEAR_TRACK 제거 → 2단계 상태머신 (PRE_TRACK → SESSION_ACTIVE)
  - 트랙 JSON 포맷 + SD 로더 + 빌트인 폴백
  - expectedLapTimeMs 제거 (dead code)

---

## 📋 계획됨 (높은 우선순위)

### P1 - 높음 (다음)
- **저장소/설정 리팩토링** - finish_line.bin 삭제, settings.json 통합, NEAR_TRACK 제거
- **트랙 JSON 포맷** - SD JSON 트랙 파일 + 빌트인 폴백, 피니시라인 기반 트랙 식별
- **GPS 테스트 페이지** - LCD에서 GPS 진단 확인 (좌표, 위성, Hz, HDOP)

### P2 - 중간
- **터치 UI 설정 메뉴** - 터치를 통한 설정 구성
- **OTA 업데이트** - 무선 펌웨어 업데이트
- **GPS UART 이벤트 큐 전환** - 폴링 → 이벤트 큐 기반 (2-5mA 절약)

---

## 🐛 알려진 문제

### 해결 완료 (2026-02-16 리팩토링)
- ~~전원 관리 비활성화~~ → PM DFS + Tickless Idle 활성화
- ~~WiFi 상시 켜짐~~ → WiFi 라이프사이클 관리 (시작 시 ON, 레이스 진입 시 OFF)
- ~~데드 코드~~ → sector.cpp/h 삭제, 미구현 선언 제거, 컴파일러 경고 0
- ~~lframe 레이스 컨디션~~ → 분석 결과 실제 race condition 아님 (동일 코어)
- ~~LVGL 플러시 200ms~~ → 50ms로 변경

### 남은 이슈

#### 빌드 환경 (Windows)
- ⚠️ Git Bash에서 `idf.py` 실패 (MSys/Mingw 감지)
  - **해결책:** ESP-IDF CMD 또는 직접 Ninja 사용
  - **문서화됨:** [../reference/build-environment.md](../reference/build-environment.md) 섹션 10.1

#### GPS 중장기 개선
- ℹ️ **UART 이벤트 큐** — 여전히 폴링 방식, 2-5mA 절약 가능
- ℹ️ **iTOW 기반 랩타이밍** — 정밀도 향상 가능, rollover 처리 필요

#### 미사용 함수/변수 (경고 아닌 잔여)
- `finish_line.cpp`: `pointToLineDistance()` 미사용
- `dead_reckoning.cpp`: `elapsedSinceUpdate` 미사용
- `gps_filter.cpp`: `sumHeading` 미사용

---

## 🔍 테스트 상태

### 시뮬레이션 모드
- ✅ 부팅 시퀀스
- ✅ 랩 재생 (에버랜드 4 랩)
- ✅ 섹터 전환 (S1→S2→S3)
- ✅ 델타 계산
- ✅ 랩 완료 감지
- ✅ 베스트 랩 추적
- ✅ 시리얼 명령

### GPS 하드웨어 모드
- ⚠️ **제한된 테스트** - 트랙에서 테스트 필요
- ✅ GPS 초기화 시퀀스
- ✅ UBX NAV-PVT 파싱
- ⚠️ 실제 트랙 검증 대기 중

### 디스플레이
- ✅ LVGL 초기화 (Waveshare 예제 정렬, 콜드부트 수정됨)
- ✅ 전용 LVGL Task (Core 0, priority 2)
- ✅ 플러시 파이프라인 (RGB565 스왑, 회전, DMA)
- ✅ LVGL indev 터치 (11-byte I2C 프로토콜)
- ✅ 섹터 델타 렌더링
- ✅ 시간/속도 델타 렌더링
- ✅ 색상 코딩 바

---

## 📊 코드 메트릭

### 코드 라인 (대략)
- **main/**: ~3,000줄
- **components/**: ~5,000줄
- **총 C/C++**: ~8,000줄
- **문서화**: ~15,000줄 (재구성 후)

### 메모리 사용량
- **플래시 (펌웨어)**: ~800 KB / 1 MB (팩토리 파티션)
- **PSRAM**: ~2 MB 사용 (레퍼런스 랩 데이터, LVGL 버퍼)
- **SPIFFS**: 가변 (랩 데이터 저장소)

### 성능
- **디스플레이 업데이트**: 30 Hz (33ms)
- **GPS 업데이트**: 10 Hz (100ms)
- **시뮬레이션 업데이트**: 20 Hz (50ms)
- **델타 계산**: <1ms (최적화됨)

---

## 🎯 다음 단계

### 단기 (이번 달)
1. 📋 **저장소/설정 리팩토링** — finish_line.bin 삭제, settings.json 통합, NEAR_TRACK 제거
2. 📋 **트랙 JSON 포맷 + SD 로더** — 빌트인 오버라이드, 피니시라인 기반 트랙 식별
3. 📋 **GPS 테스트 페이지** — LCD에서 GPS 진단 확인

### 장기 (다음 분기)
1. 터치 UI 설정 메뉴
2. OTA 업데이트 지원
3. 사용자 트랙 생성 (WiFi 웹 UI)
4. Speed Mode (가속 측정)

---

## 📚 참조

- **아키텍처:** [../reference/architecture.md](../reference/architecture.md)
- **로드맵:** [../planning/roadmap.md](../planning/roadmap.md)
- **변경 로그:** [changelog.md](changelog.md)
- **과거 스냅샷:** [snapshots/](snapshots/)
- **결정사항:** [decisions/](decisions/)

---

**전체 문서:** [../README.md](../README.md)
