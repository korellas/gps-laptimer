# OTA 펌웨어 업데이트 설계

**최종 업데이트:** 2026-02-16
**상태:** 설계 (Phase A — 문서화)
**로드맵:** Phase 4.4 (v0.6.0)
**참조:** [roadmap.md](../roadmap.md) | [features-backlog.md](../features-backlog.md)

---

## 1. 개요

### 1.1 목적

트랙 현장에서 USB 케이블 없이 펌웨어를 업데이트할 수 있는 OTA 시스템.
두 가지 무선 경로와 자동화된 빌드 파이프라인을 구축한다.

### 1.2 업데이트 경로

| 경로 | 전송 방식 | 클라이언트 | 우선순위 |
|------|-----------|-----------|----------|
| **WiFi OTA** | 캡티브 포털 파일 업로드 | 스마트폰 브라우저 | P1 (주력) |
| **BLE OTA** | Web Bluetooth API | Chrome (GitHub Pages) | P1 (주력) |
| **USB 플래시** | ESP Web Tools (Web Serial) | Chrome (GitHub Pages) | P2 (보조, 초기 플래싱) |

### 1.3 자동화 파이프라인

```
개발자: git tag v0.5.0 && git push origin v0.5.0
  ↓
GitHub Actions: espressif/idf:v5.5.2 컨테이너에서 빌드
  ↓
GitHub Release: gps_laptimer.bin 자동 첨부
  ↓
GitHub Pages: 웹앱에서 Release API로 최신 바이너리 확인
  ↓
사용자: Chrome에서 BLE 연결 → 펌웨어 전송 → OTA 완료
  또는: LAPTIMER WiFi AP 접속 → 파일 업로드 → OTA 완료
```

---

## 2. 파티션 테이블 재설계

### 2.1 현재 레이아웃 (OTA 불가)

```csv
# Name,   Type, SubType, Offset,   Size,    Flags
nvs,      data, nvs,     0x9000,   0x6000,
phy_init, data, phy,     0xf000,   0x1000,
factory,  app,  factory, 0x10000,  6M,
storage,  data, spiffs,  ,         2M,
```

- factory 파티션 6MB (실제 바이너리 ~1.5MB — 과도하게 큼)
- OTA 파티션 없음, 롤백 불가

### 2.2 새 레이아웃 (OTA 지원)

```csv
# Name,     Type, SubType, Offset,    Size,     Flags
nvs,        data, nvs,     0x9000,    0x6000,
otadata,    data, ota,     0xF000,    0x2000,
phy_init,   data, phy,     0x11000,   0x1000,
ota_0,      app,  ota_0,   0x20000,   0x600000,
ota_1,      app,  ota_1,   0x620000,  0x600000,
storage,    data, spiffs,  0xC20000,  0x200000,
```

### 2.3 메모리 맵

```
0x000000 ┌─────────────────────┐
         │ Bootloader (28KB)   │ (2nd stage bootloader)
0x009000 ├─────────────────────┤
         │ NVS (24KB)          │ WiFi 설정, 사용자 데이터
0x00F000 ├─────────────────────┤
         │ otadata (8KB)       │ 활성 OTA 슬롯 + 롤백 상태
0x011000 ├─────────────────────┤
         │ phy_init (4KB)      │ RF 캘리브레이션
0x012000 ├─────────────────────┤
         │ (패딩 ~56KB)        │ ota_0 64KB 경계 정렬
0x020000 ├─────────────────────┤
         │ ota_0 (6MB)         │ 앱 슬롯 A
0x620000 ├─────────────────────┤
         │ ota_1 (6MB)         │ 앱 슬롯 B
0xC20000 ├─────────────────────┤
         │ SPIFFS (2MB)        │ 랩 데이터, 설정
0xE20000 ├─────────────────────┤
         │ (여유 ~1.9MB)       │ 향후 확장
0x1000000└─────────────────────┘ 16MB
```

### 2.4 용량 분석

| 항목 | 크기 |
|------|------|
| 현재 바이너리 | ~1.5 MB |
| NimBLE 추가 시 | ~1.7-1.8 MB |
| OTA 슬롯 크기 | 6 MB |
| **여유 공간** | **~4.2 MB (슬롯당)** |

6MB 슬롯은 현재 바이너리의 ~4배. 향후 기능 추가에도 충분한 여유.

### 2.5 마이그레이션 주의사항

- 파티션 변경 시 **전체 플래시 필요** (`idf.py erase-flash && idf.py flash`)
- SPIFFS 데이터 소실 (레퍼런스 랩 등)
- **1회성** — 이후 OTA로 업데이트 가능
- 첫 플래시는 ota_0에 기록됨

---

## 3. WiFi OTA 설계

### 3.1 기존 인프라 활용

기존 captive portal (`components/wifi_portal/`)을 확장:

| 기존 | 추가 |
|------|------|
| SoftAP "LAPTIMER" | 그대로 사용 |
| HTTP 서버 (httpd) | OTA 엔드포인트 추가 |
| WebSocket `/ws/log` | OTA 진행률 브로드캐스트 |
| 설정 API `/api/settings` | OTA API 추가 |

### 3.2 새 엔드포인트

| 메서드 | URI | 설명 |
|--------|-----|------|
| GET | `/ota` | OTA 업데이트 페이지 (HTML) |
| POST | `/api/ota/upload` | 펌웨어 바이너리 업로드 |
| GET | `/api/ota/status` | OTA 상태 JSON |

### 3.3 업로드 프로토콜

```
클라이언트 (스마트폰)                      ESP32
    │                                        │
    │  GET /ota                              │
    │ ────────────────────────────────────>   │
    │  <── OTA 페이지 HTML                   │
    │                                        │
    │  GET /api/ota/status                   │
    │ ────────────────────────────────────>   │
    │  <── {"version":"v0.4.0","state":"idle"}│
    │                                        │
    │  POST /api/ota/upload                  │
    │  Content-Length: 1574432               │
    │  [바이너리 스트림]                      │
    │ ────────────────────────────────────>   │
    │                                        │  esp_ota_begin()
    │  <── WS: {"progress":10}               │  esp_ota_write() (4KB 청크)
    │  <── WS: {"progress":20}               │  ...
    │  <── WS: {"progress":100}              │  esp_ota_end()
    │  <── {"status":"ok","reboot":true}     │  esp_ota_set_boot_partition()
    │                                        │  esp_restart()
```

### 3.4 안전 검증

업로드 시작 전:
1. `gApp.batteryPercent >= 30` 확인 (저배터리 거부)
2. `Content-Length` ≤ OTA 파티션 크기 확인
3. 이미지 헤더 매직 바이트 검증 (`ESP_IMAGE_HEADER_MAGIC = 0xE9`)

업로드 완료 후:
4. `esp_ota_end()` — SHA256 해시 자동 검증
5. `esp_ota_set_boot_partition()` — 부트 파티션 전환

리부트 후:
6. `esp_ota_mark_app_valid_cancel_rollback()` — 정상 부팅 확인

### 3.5 WiFi OTA HTML 페이지

`wifi_portal_html.h`에 임베딩. 단일 페이지 구성:
- 현재 펌웨어 버전 표시
- 파일 선택 + 업로드 버튼
- 프로그레스 바 (WebSocket으로 실시간 업데이트)
- 상태 메시지 (검증 중, 리부트 중, 오류)

---

## 4. BLE OTA 설계

### 4.1 BLE 스택 선택

**NimBLE** (Apache NimBLE, ESP-IDF 내장):
- Bluedroid 대비 ~150KB 플래시 절약
- 메모리 사용량 적음 (~30KB RAM vs ~70KB)
- ESP-IDF v5.x에서 안정적으로 지원
- Peripheral + Broadcaster 역할만 필요 (Central/Observer 비활성화)

### 4.2 GATT 서비스 구조

```
Service: OTA Service (UUID: 0000FFE0-0000-1000-8000-00805F9B34FB)
├── Characteristic: OTA Control (0xFFE1)
│   ├── Properties: Write
│   └── 용도: 명령 전송 (START, END, ABORT, VERSION)
├── Characteristic: OTA Data (0xFFE2)
│   ├── Properties: Write Without Response
│   └── 용도: 펌웨어 청크 전송 (최대 512B/write)
└── Characteristic: OTA Status (0xFFE3)
    ├── Properties: Read, Notify
    └── 용도: 진행률, 버전, 오류 상태
```

### 4.3 전송 프로토콜

**OTA Control 명령 (0xFFE1):**

| 명령 | 바이트 | 페이로드 | 설명 |
|------|--------|----------|------|
| START | 0x01 | [total_size: u32] | OTA 시작, 전체 크기 |
| END | 0x02 | — | 전송 완료, 검증 요청 |
| ABORT | 0x03 | — | OTA 취소 |
| VERSION | 0x04 | — | 현재 버전 요청 |

**OTA Status 응답 (0xFFE3, Notify):**

| 필드 | 크기 | 설명 |
|------|------|------|
| state | 1B | 0=IDLE, 1=RECEIVING, 2=VALIDATING, 3=COMPLETE, 4=ERROR |
| progress | 1B | 0-100 (%) |
| error_code | 1B | 0=없음, 1=크기초과, 2=검증실패, 3=저배터리, 4=쓰기오류 |
| version | 16B | 현재 펌웨어 버전 문자열 (null-terminated) |

**전송 흐름:**

```
Chrome (Web Bluetooth)                      ESP32 (NimBLE)
    │                                        │
    │  requestDevice({name:"LAPTIMER-OTA"})  │
    │ ────────────────────────────────────>   │  BLE 광고 중
    │  <── 연결                               │
    │                                        │
    │  Write 0xFFE1: [0x04]                  │  VERSION 요청
    │ ────────────────────────────────────>   │
    │  <── Notify 0xFFE3: {state:0,ver:"v0.4.0"} │
    │                                        │
    │  Write 0xFFE1: [0x01, size(4B)]        │  START
    │ ────────────────────────────────────>   │  esp_ota_begin()
    │  <── Notify: {state:1, progress:0}     │
    │                                        │
    │  WriteNoResp 0xFFE2: [chunk 512B]      │  esp_ota_write()
    │  WriteNoResp 0xFFE2: [chunk 512B]      │  ...
    │  ...                                   │
    │  <── Notify: {state:1, progress:50}    │  (매 5% 알림)
    │  ...                                   │
    │                                        │
    │  Write 0xFFE1: [0x02]                  │  END
    │ ────────────────────────────────────>   │  esp_ota_end()
    │  <── Notify: {state:2, progress:100}   │  검증 중
    │  <── Notify: {state:3, progress:100}   │  완료
    │                                        │  esp_restart()
```

### 4.4 BLE 설정

```
광고 이름:       "LAPTIMER-OTA"
MTU:             517 (512B 데이터 + 3B ATT 헤더)
연결 인터벌:     7.5ms (최소, 높은 처리량)
슈퍼비전 타임아웃: 5초 (OTA 중 연결 유지)
```

### 4.5 예상 성능

| 항목 | 값 |
|------|-----|
| MTU | 512 bytes |
| 연결 인터벌 | 7.5ms |
| 예상 처리량 | ~10-15 KB/s |
| 1.5MB 펌웨어 전송 시간 | **~100-150초 (~2분)** |

### 4.6 라디오 배타성

WiFi와 BLE는 ESP32-S3의 단일 2.4GHz 라디오를 공유 (시분할 다중화).
OTA 전송 안정성을 위해:

- **BLE OTA 시작 시**: `stopWifiPortal()` 호출
- **WiFi OTA 시작 시**: BLE 비활성 상태 유지
- **OTA 완료 + 리부트 후**: 정상 모드 복귀

### 4.7 온디맨드 시작

BLE 스택은 부팅 시 로드하지 않음 (메모리/전력 절약).
시작 방법:
- 시리얼 명령어 `b` (BLE OTA 모드 토글)
- 터치 UI 메뉴 (Phase 3 이후)
- WiFi 캡티브 포털 설정 페이지

---

## 5. 버전 관리

### 5.1 현재

```cpp
// components/common/config.h:189
constexpr const char* APP_VERSION = "v0.4.0";
```

수동으로 하드코딩. 빌드마다 동일.

### 5.2 개선: 태그 기반 자동 주입

**CMake (프로젝트 루트):**
```cmake
# CMakeLists.txt에 추가
execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_VERSION)
    set(GIT_VERSION "v0.0.0-dev")
endif()
```

**config.h 변경:**
```cpp
#ifdef APP_VERSION_FROM_GIT
constexpr const char* APP_VERSION = APP_VERSION_FROM_GIT;
#else
constexpr const char* APP_VERSION = "v0.0.0-dev";
#endif
```

**GitHub Actions에서:**
```yaml
- name: Build with version
  run: |
    VERSION=${GITHUB_REF_NAME}  # "v0.5.0"
    idf.py build -DAPP_VERSION_FROM_GIT=\"${VERSION}\"
```

### 5.3 버전 체계

```
v{MAJOR}.{MINOR}.{PATCH}[-{PRERELEASE}]

예시:
  v0.4.0       — Phase 2 릴리즈
  v0.5.0       — Phase 3 릴리즈
  v0.5.1       — 패치 릴리즈
  v0.5.0-rc1   — 릴리즈 후보
  v0.0.0-dev   — 개발 빌드 (태그 없음)
```

### 5.4 OTA 버전 확인

OTA 시작 시:
- 현재 실행 중인 버전: `APP_VERSION` 또는 `esp_ota_get_app_description()->version`
- 업로드되는 바이너리 버전: 이미지 헤더의 `esp_app_desc_t.version`
- **다운그레이드 허용** (롤백 시나리오)
- **동일 버전 허용** (재설치)

---

## 6. GitHub Actions CI/CD

### 6.1 릴리즈 워크플로우

**파일:** `.github/workflows/release.yml`

```yaml
name: Build & Release Firmware

on:
  push:
    tags:
      - 'v*'

jobs:
  build:
    runs-on: ubuntu-latest
    container:
      image: espressif/idf:v5.5.2

    steps:
      - uses: actions/checkout@v4

      - name: Inject version from tag
        run: |
          VERSION=${GITHUB_REF_NAME}
          echo "Building version: ${VERSION}"

      - name: Build firmware
        run: |
          idf.py build -DAPP_VERSION_FROM_GIT=\"${GITHUB_REF_NAME}\"

      - name: Create Release
        uses: softprops/action-gh-release@v2
        with:
          generate_release_notes: true
          files: |
            build/gps_laptimer.bin
            build/bootloader/bootloader.bin
            build/partition_table/partition-table.bin
```

### 6.2 빌드 아티팩트

| 파일 | 용도 |
|------|------|
| `gps_laptimer.bin` | **OTA 업데이트용** (앱 파티션만) |
| `bootloader.bin` | 전체 플래시용 (부트로더) |
| `partition-table.bin` | 전체 플래시용 (파티션 테이블) |

OTA는 `gps_laptimer.bin`만 사용. 나머지는 초기 플래시 또는 파티션 변경 시 사용.

### 6.3 태그 워크플로우

```bash
# 릴리즈 준비
git checkout main
git pull

# 태그 생성 (annotated)
git tag -a v0.5.0 -m "섹터 타이밍, BLE OTA 지원"

# 리모트에 push → Actions 자동 실행
git push origin v0.5.0
```

---

## 7. GitHub Pages 웹앱

### 7.1 아키텍처

```
GitHub Pages (HTTPS)
├── index.html          ← 메인 페이지
├── ota.js              ← Web Bluetooth + OTA 로직
└── style.css           ← 스타일

외부 의존성: 없음 (Vanilla JS)
빌드 도구: 없음
```

### 7.2 기능

1. **장치 검색**: `navigator.bluetooth.requestDevice()` — "LAPTIMER" 필터
2. **버전 확인**: BLE로 현재 디바이스 버전 읽기
3. **펌웨어 다운로드**: GitHub Releases API → 최신 `.bin` fetch
4. **BLE 전송**: 512B 청크로 Write Without Response
5. **진행률 표시**: 프로그레스 바 + 예상 남은 시간
6. **수동 업로드**: 로컬 `.bin` 파일 선택 (오프라인 사용)

### 7.3 브라우저 호환성

| 브라우저 | Web Bluetooth | Web Serial (USB) |
|---------|---------------|-------------------|
| Chrome (데스크톱) | O | O |
| Chrome (Android) | O | X |
| Edge | O | O |
| Firefox | X | X |
| Safari | X | X |

**최소 요구사항:** Chrome 56+ 또는 Edge 79+

### 7.4 GitHub Pages 배포

소스: `web/` 디렉토리 또는 별도 `gh-pages` 브랜치.
GitHub Actions로 자동 배포 가능:

```yaml
# .github/workflows/pages.yml
on:
  push:
    branches: [main]
    paths: ['web/**']

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: peaceiris/actions-gh-pages@v4
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./web
```

---

## 8. 안전장치

### 8.1 배터리 보호

```
OTA 시작 전: gApp.batteryPercent >= 30% 확인
→ 미달 시 거부 + 오류 메시지 ("배터리 부족")
```

### 8.2 롤백 지원

ESP-IDF 자동 롤백 (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`):

```
정상 흐름:
1. OTA 기록 → ota_1 (비활성 슬롯)
2. 부트 파티션 → ota_1로 전환
3. 리부트
4. ota_1에서 부팅 (pending verify 상태)
5. 디스플레이 + GPS 초기화 성공 확인
6. esp_ota_mark_app_valid_cancel_rollback()
7. 정상 운영

실패 흐름:
1-4. 동일
5. 부팅 중 크래시 또는 타임아웃
6. 다음 부팅 시 부트로더가 ota_0 (이전 버전)으로 자동 롤백
```

### 8.3 이미지 검증

- **헤더 매직**: `0xE9` (ESP32 앱 이미지)
- **SHA256**: `esp_ota_end()`가 자동으로 검증
- **칩 호환성**: 이미지 헤더의 chip_id 확인 (ESP32-S3 = 0x0009)

### 8.4 OTA 중 동작 제한

OTA 진행 중:
- GPS 모드 일시정지 (시뮬레이션/하드웨어 모두)
- 디스플레이: OTA 진행 화면으로 전환
- 시리얼 명령: OTA 관련만 허용 (상태, 취소)
- WiFi/BLE 라디오: 한 쪽만 활성

---

## 9. LCD OTA 진행 화면

### 9.1 레이아웃

```
┌────────────────────────────────────────────────────────────┐
│  FIRMWARE UPDATE                      v0.4.0 → v0.5.0     │
│                                                            │
│  [████████████████████░░░░░░░░░░]  67%                    │
│  1.05 MB / 1.57 MB                    DO NOT POWER OFF     │
└────────────────────────────────────────────────────────────┘
```

### 9.2 상태별 표시

| 상태 | 표시 |
|------|------|
| IDLE | (화면 표시 안함 — 정상 모드) |
| RECEIVING | 프로그레스 바 + 수신 바이트 |
| VALIDATING | "Verifying firmware..." |
| APPLYING | "Applying update..." |
| COMPLETE | "Update complete! Rebooting..." (3초 후 자동 리부트) |
| ERROR | 오류 메시지 + "Press any key to return" |

---

## 10. sdkconfig 변경사항

### 10.1 NimBLE 활성화

```
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1
CONFIG_BT_NIMBLE_ROLE_CENTRAL=n
CONFIG_BT_NIMBLE_ROLE_OBSERVER=n
CONFIG_BT_NIMBLE_ROLE_BROADCASTER=y
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y
CONFIG_BT_NIMBLE_NVS_PERSIST=n
CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT=n
```

### 10.2 OTA/롤백

```
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_APP_ROLLBACK_ENABLE=y
```

### 10.3 파티션

```
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

(이미 설정됨 — 파티션 테이블 파일 내용만 변경)

---

## 11. 컴포넌트 구조

### 11.1 새 컴포넌트

```
components/
├── ota/                    ← OTA 코어 (WiFi/BLE 공통)
│   ├── CMakeLists.txt
│   ├── ota_manager.cpp
│   └── ota_manager.h
└── ble_ota/                ← BLE OTA 전용
    ├── CMakeLists.txt
    ├── ble_ota.cpp
    └── ble_ota.h
```

### 11.2 OTA Manager API

```cpp
// ota_manager.h
namespace ota {

enum class State { IDLE, RECEIVING, VALIDATING, APPLYING, COMPLETE, ERROR };

struct Status {
    State state;
    float progress;          // 0.0 - 1.0
    uint32_t receivedBytes;
    uint32_t totalBytes;
    char errorMsg[64];
    char currentVersion[32];
    char newVersion[32];
};

bool begin(uint32_t imageSize);
bool write(const uint8_t* data, size_t len);
bool end();
void abort();
void confirmBoot();          // 부팅 시 호출
const Status& getStatus();

} // namespace ota
```

### 11.3 BLE OTA API

```cpp
// ble_ota.h
namespace ble_ota {

void init();       // NimBLE 초기화 (1회)
void start();      // 광고 시작, GATT 서비스 등록
void stop();       // 광고 중지, BLE 비활성화
bool isActive();

} // namespace ble_ota
```

---

## 12. 구현 순서

```
Phase B: GitHub Actions + 버전 관리
  ├── .github/workflows/release.yml
  ├── CMakeLists.txt (버전 주입)
  └── config.h (조건부 APP_VERSION)
         ↓
Phase C: 파티션 + OTA 코어
  ├── partitions.csv (새 레이아웃)
  ├── sdkconfig.defaults (롤백 활성화)
  ├── components/ota/ (ota_manager)
  ├── types.h (OTA 상태 필드)
  ├── main.cpp (부팅 시 롤백 확인)
  └── waveshare_display.cpp (OTA 진행 화면)
         ↓
     ┌───┴────────┐
Phase D: WiFi OTA    Phase E: BLE OTA
  wifi_portal.cpp      sdkconfig (NimBLE)
  wifi_portal_html.h   components/ble_ota/
                       serial_commands.cpp
                            ↓
                       Phase F: GitHub Pages 웹앱
                         web/index.html
                         web/ota.js
                         web/style.css
```

Phase D와 E는 독립적으로 진행 가능. Phase F는 E 이후.

---

## 13. 위험 및 완화

| 위험 | 영향 | 확률 | 완화 |
|------|------|------|------|
| 파티션 변경 시 데이터 소실 | 높음 | 확정 | 마이그레이션 전 백업 안내 |
| OTA 중 전원 차단 | 치명적 | 낮음 | 롤백 + 배터리 체크 |
| NimBLE 바이너리 크기 초과 | 중간 | 낮음 | 3MB 슬롯으로 충분한 여유 |
| BLE 전송 중 연결 끊김 | 중간 | 중간 | abort + 재시도 가능 |
| Web Bluetooth 브라우저 미지원 | 낮음 | 확정 | WiFi OTA를 대안으로 제공 |
| GitHub Actions 빌드 환경 불일치 | 낮음 | 낮음 | ESP-IDF Docker 이미지 버전 고정 |

---

## 14. 참조

- [ESP-IDF OTA API (ESP32-S3)](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/api-reference/system/ota.html)
- [ESP-IDF NimBLE](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/api-reference/bluetooth/nimble/index.html)
- [Web Bluetooth API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API)
- [SparkFun BLE OTA Tutorial](https://learn.sparkfun.com/tutorials/esp32-ota-updates-over-ble-from-a-react-web-application/all)
- [ESP-IDF BLE OTA (Bluedroid)](https://github.com/AvinasheeTech/ESP32-IDF-BLE-OTA)
- [softprops/action-gh-release](https://github.com/softprops/action-gh-release)
