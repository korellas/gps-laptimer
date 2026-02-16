# 빠른 시작 가이드

**GPS 랩 타이머 - 15분 안에 실행하기**

이 가이드는 Waveshare ESP32-S3-Touch-LCD-3.49에서 GPS 랩 타이머를 처음부터 실행하는 방법을 안내합니다.

---

## 사전 준비사항

시작하기 전에 다음을 확인하세요:

- [ ] **하드웨어:**
  - Waveshare ESP32-S3-Touch-LCD-3.49 보드
  - USB-C 케이블 (데이터 + 전원)
  - 컴퓨터 (Windows/Linux/macOS)
  - 선택사항: u-blox G10A-F33 GPS 모듈

- [ ] **소프트웨어:**
  - ESP-IDF v5.5.2 설치됨
  - Git 설치됨
  - Python 3.x 설치됨 (보통 ESP-IDF와 함께 제공)

---

## 단계 1: ESP-IDF 설치 (15분)

### Windows

1. **ESP-IDF 설치 프로그램 다운로드:**
   - 방문: https://dl.espressif.com/dl/esp-idf/
   - 다운로드: `esp-idf-tools-setup-x.x.x.exe`

2. **설치 프로그램 실행:**
   - ESP-IDF v5.5.2 선택
   - 기본 설치 경로: `C:\Espressif\`
   - 설치 대기 (~10분)

3. **설치 확인:**
   ```cmd
   # 시작 메뉴에서 "ESP-IDF 5.5 CMD" 열기
   idf.py --version
   # 표시되어야 함: ESP-IDF v5.5.2
   ```

**전체 지침:** [../reference/build-environment.md](../reference/build-environment.md)

### Linux/macOS

```bash
# 사전 준비사항 설치
sudo apt-get install git wget flex bison gperf python3 python3-pip \
  python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util

# ESP-IDF 클론
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.2

# 도구 설치
./install.sh esp32s3

# 환경 설정 (~/.bashrc에 추가)
. ~/esp/esp-idf/export.sh
```

---

## 단계 2: 저장소 클론 (1분)

```bash
# 프로젝트 클론
git clone https://github.com/YOUR-USERNAME/gps_laptimer.git
cd gps_laptimer

# 또는 프로젝트가 이미 있는 경우, 해당 위치로 이동
cd /path/to/gps_laptimer
```

---

## 단계 3: 펌웨어 빌드 (5분)

### Windows (ESP-IDF CMD)

```cmd
# 타겟 설정 (처음만)
idf.py set-target esp32s3

# 빌드
idf.py build

# 첫 빌드는 3-5분 소요
# 이후 빌드는 더 빠름 (~30초)
```

### Linux/macOS

```bash
# ESP-IDF 환경 활성화
. ~/esp/esp-idf/export.sh

# 타겟 설정 (처음만)
idf.py set-target esp32s3

# 빌드
idf.py build
```

**예상 출력:**
```
...
Project build complete. To flash, run:
 idf.py -p (PORT) flash
or
 idf.py -p (PORT) flash monitor
```

**문제 해결:**
- **에러: "idf.py not found"** → ESP-IDF 환경이 활성화되지 않음
- **에러: "espressif__* not found"** → `idf.py reconfigure` 실행하여 의존성 다운로드
- **Windows Git Bash 실패** → ESP-IDF CMD 사용 (참조 [build-environment.md](../reference/build-environment.md) 섹션 10.1)

---

## 단계 4: 하드웨어 연결 (30초)

1. **USB-C 케이블 연결** 컴퓨터에서 ESP32-S3 보드로
2. **COM 포트 찾기:**
   - **Windows:** 장치 관리자 → 포트 → `USB Serial Device (COMX)`
   - **Linux:** `ls /dev/ttyUSB*` 또는 `ls /dev/ttyACM*`
   - **macOS:** `ls /dev/cu.*`
3. **포트 기록** (예: `COM3`, `/dev/ttyUSB0`)

---

## 단계 5: 펌웨어 플래시 (2분)

```bash
# COM3을 본인의 포트로 교체
idf.py -p COM3 flash monitor

# 또는 플래시만 (모니터 없이)
idf.py -p COM3 flash
```

**예상 출력:**
```
Connecting.....
Writing at 0x00010000... (10%)
Writing at 0x00020000... (20%)
...
Hash of data verified.
Leaving...
Hard resetting via RTS pin...
```

**문제 해결:**
- **에러: "Failed to connect"** → 연결하는 동안 BOOT 버튼 누르기
- **에러: "Permission denied"** → `dialout` 그룹에 사용자 추가 (Linux) 또는 관리자 권한 실행 (Windows)
- **에러: "No serial port found"** → USB 케이블 확인 (데이터 케이블이어야 함, 충전 전용 아님)

---

## 단계 6: 첫 실행 (1분)

플래시 후, 장치가 다음을 수행해야 합니다:

1. **부팅** (~2초)
2. **디스플레이 초기화** (화면이 켜지는 것을 볼 수 있음)
3. **시뮬레이션 모드 시작** (에버랜드 트랙 재생)

**예상 시리얼 출력:**
```
ESP-ROM:esp32s3-20210327
...
I (123) main: GPS Lap Timer starting...
I (234) display: Display initialized (640x172)
I (345) LVGL: LVGL initialized
I (456) SectorTiming: Initialized with 3 sectors
I (567) main: Mode: SIMULATION
I (678) simulation: Starting Everland simulation...
```

**보여야 할 것:**
- **디스플레이:** "LAP 01", 델타 타임, 섹터 델타가 있는 UI
- **시리얼:** 시뮬레이션 진행 상황을 보여주는 로그

---

## 단계 7: 시리얼을 통한 상호작용 (2분)

### 시리얼 모니터 열기

```bash
# 단계 5에서 "flash monitor"를 실행했다면, 이미 모니터에 있음
# 그렇지 않으면, 시작:
idf.py -p COM3 monitor

# 모니터 종료: Ctrl+]
```

### 명령 시도

| 명령 | 수행되는 작업 |
|---------|--------------|
| `s` | 상태 표시 (모드, 랩, GPS 정보) |
| `h` | 도움말 표시 (모든 명령) |
| `e` | GPS ↔ 시뮬레이션 모드 토글 |
| `r` | 현재 랩 리셋 |

**예시:**
```
> s
Mode: SIMULATION
Lap: 01
Time: 12.34s
Delta: +1.23s
Sector: S2
GPS: Simulated

> e
Switched to GPS_HARDWARE mode

> e
Switched to SIMULATION mode
```

---

## 단계 8: 선택사항 - GPS 연결 (10분)

u-blox G10A-F33 GPS 모듈이 있는 경우:

### 배선

| GPS 모듈 핀 | ESP32-S3 핀 | 선 색상 |
|-------------|-------------|---------|
| PPS | GPIO1 | 하양 (white) |
| VCC | 3.3V | 빨강 (red) |
| TX | GPIO44 (UART2 RX) | 파랑 (blue) |
| RX | GPIO43 (UART2 TX) | 초록 (green) |
| GND | GND | 검정 (black) |
| ENABLE | GPIO0 | 노랑 (yellow) |

**참조:** [../reference/hardware.md](../reference/hardware.md) 섹션 4.1

### GPS 테스트

1. **GPS 모드로 전환:**
   ```
   > e
   Switched to GPS_HARDWARE mode
   ```

2. **GPS fix 대기** (야외에서 30초 ~ 2분)
   ```
   I (1234) GPS: Searching for satellites...
   I (5678) GPS: Fix acquired: 3D, 8 satellites
   I (9012) GPS: Position: 37.296096, 127.206860
   ```

3. **상태 확인:**
   ```
   > s
   Mode: GPS_HARDWARE
   GPS: 3D Fix, 8 satellites
   Position: 37.296096, 127.206860
   Speed: 0.0 km/h
   ```

**문제 해결:**
- **위성 없음** → 야외로 이동, 더 오래 대기 (콜드 스타트 ~30초)
- **배선 문제** → 연결 확인, 3.3V 확인 (5V 아님!)
- **잘못된 데이터** → TX/RX가 바뀌지 않았는지 확인

---

## 단계 9: UI 이해하기

```
┌────────────────────────────────────────────────────────────┐
│  S1 -0.42       LAP 01                1:23.4              │
│  S2 +0.72                           BEST 1:27:00          │
│  S3 -0.32        +1.34                                    │
│  ████████████████████████                    +1.2km/h     │
└────────────────────────────────────────────────────────────┘
```

| 요소 | 의미 |
|---------|---------|
| **S1/S2/S3** | 섹터 델타 (음수 = 더 빠름, 양수 = 더 느림) |
| **LAP 01** | 현재 랩 번호 |
| **1:23.4** | 현재 랩 타임 (분:초.1/10초) |
| **BEST 1:27:00** | 이 세션의 베스트 랩 타임 |
| **+1.34** | 레퍼런스 랩 대비 시간 델타 (중앙, 큼) |
| **바 (하단)** | 속도 델타 (녹색 = 더 빠름, 빨강 = 더 느림) |
| **+1.2km/h** | 속도 델타 텍스트 (바 가장자리에서 이동) |

**전체 사양:** [../reference/ui-specification.md](../reference/ui-specification.md)

---

## 단계 10: 다음 단계

축하합니다! GPS 랩 타이머가 실행 중입니다. 🎉

### 더 알아보기

- **아키텍처:** [../reference/architecture.md](../reference/architecture.md)
- **시리얼 명령:** [../CLAUDE.md](../../CLAUDE.md) 섹션 "시리얼 명령"
- **디버깅:** [debugging.md](debugging.md)

### 시도해보기

1. **시뮬레이션 랩 완료 보기:**
   - 랩 완료까지 ~52초 대기
   - "LAP COMPLETE" 오버레이 보기
   - 새 랩 자동 시작

2. **결승선 설정 테스트 (GPS 모드만):**
   ```
   > e             # GPS 모드로 전환
   > f             # 현재 위치에 결승선 설정
   > s             # 결승선 설정 확인
   ```

3. **랩 저장 및 로드:**
   ```
   > l             # 저장된 랩 목록
   > n             # 새 세션 시작
   ```

### 커스터마이징

- **트랙 변경:** [다중 트랙 지원](../planning/roadmap.md) 참조 (계획됨)
- **레퍼런스 랩 추가:** [../reference/tracks/README.md](../reference/tracks/README.md) 참조
- **UI 수정:** [../reference/ui-specification.md](../reference/ui-specification.md) 참조

---

## 일반적인 문제

### 빌드 문제

**문제:** `idf.py: command not found`
**해결책:** ESP-IDF 환경이 활성화되지 않음. `export.sh` (Linux/macOS) 실행 또는 ESP-IDF CMD (Windows) 사용

**문제:** "espressif__* component not found"로 빌드 실패
**해결책:** `idf.py reconfigure` 실행하여 관리되는 컴포넌트 다운로드

**문제:** Windows Git Bash `idf.py` 실패
**해결책:** ESP-IDF CMD 사용 또는 직접 Ninja 빌드 (참조 [build-environment.md](../reference/build-environment.md) 섹션 10.1)

### 플래시 문제

**문제:** "Failed to connect to ESP32-S3"
**해결책:** 플래시 명령 실행 중 BOOT 버튼 누르기

**문제:** Linux에서 "Permission denied"
**해결책:** `sudo usermod -a -G dialout $USER` 후 로그아웃/로그인

**문제:** 플래시 성공했지만 장치 부팅 안 됨
**해결책:** `idf.py -p COM3 erase-flash` 실행 후 다시 플래시

### 디스플레이 문제

**문제:** 디스플레이가 비어있음
**해결책:** 디스플레이 초기화 에러에 대한 시리얼 로그 확인. 보드가 정품 Waveshare인지 확인.

**문제:** 디스플레이가 잘못 회전됨
**해결책:** 소프트웨어 회전이 코드에 고정됨 (참조 [hardware.md](../reference/hardware.md))

### GPS 문제

**문제:** GPS fix 없음
**해결책:** 야외로 이동, 30-60초 대기. 배선 확인 (TX→RX44, RX→TX43).

**문제:** 잘못된 좌표
**해결책:** TX/RX가 바뀌지 않았는지 확인. 보드레이트 확인 (9600).

---

## 도움 받기

1. **문서 확인:**
   - [문서 인덱스](../README.md)
   - [참조 문서](../reference/)
   - [CLAUDE.md](../../CLAUDE.md)

2. **로그 확인:**
   ```bash
   idf.py -p COM3 monitor
   # 에러/경고 찾기
   ```

3. **이슈 제출:**
   - 포함사항: 보드 버전, ESP-IDF 버전, 에러 로그
   - GitHub: [프로젝트 이슈](https://github.com/YOUR-USERNAME/gps_laptimer/issues)

---

## 요약

성공적으로:
- ✅ ESP-IDF 설치
- ✅ 펌웨어 빌드
- ✅ 장치 플래시
- ✅ 시뮬레이션 모드 테스트
- ✅ (선택사항) GPS 연결

**총 시간:** ~15-30분

**다음:** [기능 로드맵](../planning/roadmap.md) 탐색 및 [기여](../../README.md#contributing)!

---

**문서로 돌아가기:** [../README.md](../README.md)
