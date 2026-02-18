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
- 근거 파일:
  - `components/modes/gps_processor.cpp`
  - `main/finish_line.cpp`
- 제안:
  - 트랙/레이아웃 후보별로 최초 1회만 `setFinishLineFromDefinition()` 호출
  - 동일 후보에 대해 crossing state 유지
  - 후보 변경 시에만 crossing state reset
- 기대 효과: 자동 세션 진입 실패/불안정 감소, 실주행 시작 판정 신뢰도 향상

### 2. OTA 경로 보안 강화 (Wi-Fi/BLE 공통)
- 우선순위: High
- 문제: OTA 업로드 경로에 인증/권한 제어가 약하고, BLE OTA도 보안 플래그 없이 쓰기 허용 범위가 큼.
- 근거 파일:
  - `components/wifi_portal/wifi_portal.cpp`
  - `components/ble_ota/ble_ota.cpp`
- 제안:
  - Wi-Fi AP 인증 도입(최소 WPA2) 또는 OTA 페이지 토큰 인증 추가
  - OTA 요청에 세션 토큰/nonce 검증 적용
  - BLE characteristic 권한/암호화 요구사항 적용
  - 가능하면 이미지 서명 검증 정책 문서화
- 기대 효과: 무단 펌웨어 업로드/변조 리스크 대폭 감소

### 3. 저장 파일 로드 시 pointCount 상한 검증
- 우선순위: High
- 문제: 손상 파일 헤더의 `pointCount`를 그대로 신뢰해 대규모 할당이 발생할 수 있음.
- 근거 파일:
  - `main/lap_storage.cpp`
  - `main/lap_storage.h`
- 제안:
  - `header.pointCount == 0 || header.pointCount > MAX_POINTS_PER_LAP` 즉시 실패 처리
  - 파일 길이와 헤더 기반 기대 길이 일치 검증 추가
  - 로드 실패 로그에 원인 코드 명시
- 기대 효과: 메모리 보호, 비정상 파일 입력 내구성 향상

### 4. 모듈 경계 정리 (`components`가 `main`을 참조하는 구조)
- 우선순위: Medium
- 문제: `components/*.h`에서 `../main/*.h`를 include하는 브리지 구조가 증가해 계층이 역전됨.
- 근거 파일:
  - `components/lap_storage.h`
  - `components/ublox_gps.h`
  - `components/waveshare_display.h`
  - `components/protocol.hpp`
- 제안:
  - 공용 인터페이스는 `components/common` 또는 각 컴포넌트 내부 헤더로 이동
  - `main/`는 조립/엔트리만 담당하도록 역할 축소
  - include 경로를 컴포넌트 단위로 정리
- 기대 효과: 의존성 추적 단순화, 테스트/교체/재사용성 향상

### 5. 미사용/부분구현 모듈 정리 (`lap_manager`)
- 우선순위: Medium
- 문제: API는 많지만 실제 호출이 거의 없고 TODO가 남아 유지보수 비용만 증가.
- 근거 파일:
  - `components/timing/lap_manager.cpp`
  - `components/timing/lap_manager.h`
- 제안:
  - 선택지 A: 현재 아키텍처에 맞춰 실제 사용 경로로 통합
  - 선택지 B: 사용 중단 모듈로 명시 후 제거
  - TODO(`loadRefLapFromStorage`, `saveAsReferenceLap`) 처리 방향 확정
- 기대 효과: 중복 책임 제거, 코드 탐색 비용 감소

### 6. 델타 계산 엣지케이스 보정
- 우선순위: Medium
- 문제: `getReferenceTimeAtDistance()` 결과 0ms를 실패로 간주하는 로직이 랩 시작점과 충돌 가능.
- 근거 파일:
  - `components/timing/delta_calculator.cpp`
- 제안:
  - 성공/실패를 시간값(0) 대신 bool/상태코드로 분리
  - unsigned 차감 대신 음수 가능 타입으로 안전 계산
  - 거리=0, 랩 시작/종료 경계 단위 테스트 추가
- 기대 효과: 시작 구간 델타 안정성 개선, 잠재 언더플로우 방지

### 7. 대형 파일 분할 (`waveshare_display.cpp`)
- 우선순위: Low (구조 개선)
- 문제: LCD 드라이버, LVGL 렌더링, UI 업데이트, 전원 제어가 한 파일에 과집중.
- 근거 파일:
  - `main/waveshare_display.cpp`
- 제안:
  - `display_hal`(패널/터치/백라이트)
  - `display_renderer`(lap/gps/time render)
  - `display_power`(shutdown/backlight)
  - `display_widgets_api`(외부 노출)로 분리
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
