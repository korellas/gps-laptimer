# 빌드 환경

**상태:** 2026-02-14 기준 최신
**SSOT:** 빌드 구성 및 절차의 단일 정보원

## 1. 빌드 시스템

| 구성 요소 | 버전 | 경로 |
|----------|------|------|
| **ESP-IDF** | v5.5.2 | `c:\esp\v5.5.2\esp-idf` (Windows) |
| **빌드 시스템** | CMake + Ninja | ESP-IDF 통합 |
| **툴체인** | xtensa-esp-elf-gcc | ESP-IDF 관리 |
| **Python** | 3.x | ESP-IDF venv |

## 2. 설치

### 2.1 Windows

**방법 1: ESP-IDF Tools 설치 프로그램 (권장)**
1. 설치 프로그램 다운로드: https://dl.espressif.com/dl/esp-idf/
2. 설치 프로그램 실행, ESP-IDF v5.5.2 선택
3. 기본 설치 경로: `C:\Espressif\`
4. 설치 프로그램이 설정하는 항목:
   - ESP-IDF 프레임워크
   - Python 가상 환경
   - 툴체인 (xtensa-esp-elf, riscv32-esp-elf)
   - 빌드 도구 (CMake, Ninja)

**방법 2: 수동 설치**
참조: https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/get-started/windows-setup.html

### 2.2 Linux/macOS

```bash
# 필수 구성 요소 설치
sudo apt-get install git wget flex bison gperf python3 python3-pip \
  python3-venv cmake ninja-build ccache libffi-dev libssl-dev \
  dfu-util libusb-1.0-0

# ESP-IDF 클론
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.5.2

# 도구 설치
./install.sh esp32s3

# 환경 설정
. ./export.sh
```

## 3. 환경 설정

### 3.1 Windows (PowerShell)

```powershell
$env:IDF_PATH = 'C:\esp\v5.5.2\esp-idf'
$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v5.5.2\venv'
$env:PATH = 'C:\Espressif\tools\cmake\3.30.2\bin;' +
            'C:\Espressif\tools\ninja\1.12.1;' +
            'C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20251107\xtensa-esp-elf\bin;' +
            'C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20251107\riscv32-esp-elf\bin;' +
            'C:\Espressif\tools\python\v5.5.2\venv\Scripts;' +
            $env:PATH
```

**또는 ESP-IDF 명령 프롬프트 사용:**
- 시작 메뉴 → ESP-IDF → ESP-IDF 5.5 CMD

### 3.2 Linux/macOS (Bash)

```bash
# ~/.bashrc 또는 ~/.zshrc에 추가
export IDF_PATH=~/esp/esp-idf
source $IDF_PATH/export.sh
```

## 4. 프로젝트 구성

### 4.1 타겟 선택

```bash
# 타겟 설정 (처음 한 번만)
idf.py set-target esp32s3
```

이 명령은 ESP32-S3 기본값으로 `sdkconfig`를 생성합니다.

### 4.2 SDK 구성

**파일:** `sdkconfig.defaults`

주요 구성:
```ini
# ESP32-S3 전용
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y

# PSRAM
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM=y

# USB Serial/JTAG 콘솔
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y

# LVGL
CONFIG_LV_USE_DRAW_SW=y
CONFIG_LV_COLOR_DEPTH_16=y

# FreeRTOS
CONFIG_FREERTOS_HZ=1000
```

**전체 구성:**
```bash
idf.py menuconfig
```

### 4.3 파티션 테이블

**파일:** `partitions.csv`

```csv
# Name,     Type, SubType, Offset,  Size,    Flags
nvs,        data, nvs,     0x9000,  0x6000,
phy_init,   data, phy,     0xf000,  0x1000,
factory,    app,  factory, 0x10000, 1M,
spiffs,     data, spiffs,  ,        1M,
```

| 파티션 | 크기 | 용도 |
|--------|------|------|
| **nvs** | 24 KB | 비휘발성 저장소 (WiFi, 설정) |
| **phy_init** | 4 KB | RF 보정 데이터 |
| **factory** | 1 MB | 애플리케이션 펌웨어 |
| **spiffs** | 1 MB | 파일 시스템 (랩 데이터, 폰트) |

## 5. 빌드 명령

### 5.1 표준 빌드

```bash
# 클린 빌드
idf.py fullclean
idf.py build

# 증분 빌드
idf.py build
```

### 5.2 빌드 출력

```
build/
├── gps_laptimer.bin       # 플래시 가능한 바이너리
├── gps_laptimer.elf       # ELF 파일 (디버깅용)
├── gps_laptimer.map       # 메모리 맵
├── bootloader/
│   └── bootloader.bin
└── partition_table/
    └── partition-table.bin
```

### 5.3 빌드 크기 분석

```bash
idf.py size
idf.py size-components
idf.py size-files
```

## 6. 플래싱

### 6.1 펌웨어 플래시

```bash
# 플래시 및 모니터 (권장)
idf.py -p COM3 flash monitor

# 플래시만
idf.py -p COM3 flash

# 먼저 플래시 지우기 (클린 시작)
idf.py -p COM3 erase-flash
idf.py -p COM3 flash
```

### 6.2 포트 선택

**Windows:**
- 장치 관리자 → 포트(COM 및 LPT) 확인
- 일반적으로: `COM3`, `COM4` 등

**Linux:**
- 확인: `ls /dev/ttyUSB* /dev/ttyACM*`
- 일반적으로: `/dev/ttyUSB0` 또는 `/dev/ttyACM0`

**macOS:**
- 확인: `ls /dev/cu.*`
- 일반적으로: `/dev/cu.usbserial-*`

## 7. 모니터링

### 7.1 시리얼 모니터

```bash
# 모니터 시작 (플래시 후)
idf.py -p COM3 monitor

# 모니터만 (플래시 없음)
idf.py -p COM3 monitor

# 종료: Ctrl+]
```

### 7.2 모니터 옵션

```bash
# 사용자 지정 보드레이트 (USB 콘솔에는 불필요)
idf.py -p COM3 -b 115200 monitor

# 주소 디코드 (스택 추적)
idf.py monitor
```

모니터는 `.elf` 파일을 사용하여 패닉 백트레이스를 자동으로 디코드합니다.

## 8. 디버깅

### 8.1 디버그 빌드

```bash
# 디버그 심볼 활성화
idf.py menuconfig
# Component config → Compiler options → Optimization Level → Debug (-Og)

idf.py build
```

### 8.2 JTAG 디버깅

**하드웨어:** ESP32-S3에 내장된 USB JTAG
**도구:** OpenOCD + GDB

```bash
# OpenOCD 시작 (별도 터미널)
openocd -f board/esp32s3-builtin.cfg

# GDB (다른 터미널)
xtensa-esp32s3-elf-gdb build/gps_laptimer.elf
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) break app_main
(gdb) continue
```

### 8.3 로깅

**레벨:** Error, Warning, Info, Debug, Verbose

```cpp
#include "esp_log.h"

static const char* TAG = "MyModule";

ESP_LOGE(TAG, "Error: %d", code);
ESP_LOGW(TAG, "Warning: %s", msg);
ESP_LOGI(TAG, "Info: %.2f", value);
ESP_LOGD(TAG, "Debug: %p", ptr);
ESP_LOGV(TAG, "Verbose: %lu", count);
```

**로그 레벨 구성:**
```bash
idf.py menuconfig
# Component config → Log output → Default log verbosity
```

## 9. 종속성

### 9.1 관리 컴포넌트

**파일:** `main/idf_component.yml`

```yaml
dependencies:
  espressif/esp_lcd_axs15231b: "^1.0.0"
  espressif/esp_lcd_touch_cst816s: "^1.0.0"
  lvgl/lvgl: "^9.0.0"
```

**컴포넌트 업데이트:**
```bash
idf.py reconfigure
```

### 9.2 로컬 컴포넌트

```
components/
├── geo/
├── track/
├── timing/
├── modes/
├── app/
└── common/
```

각 컴포넌트에 `CMakeLists.txt` 있음:
```cmake
idf_component_register(
    SRCS "file1.cpp" "file2.cpp"
    INCLUDE_DIRS "."
    REQUIRES main
)
```

## 10. 알려진 문제

### 10.1 Windows 빌드 (Git Bash의 idf.py)

**문제:** Git Bash에서 `idf.py` 실패 (MSys/Mingw 감지)

**해결책:** `cmd.exe` 또는 PowerShell 사용, 또는 직접 Ninja:

```bash
# 직접 Ninja 빌드 (회피책)
cmd.exe //c "set IDF_PATH=c:\esp\v5.5.2\esp-idf && set MSYSTEM= && \
  set IDF_PYTHON_ENV_PATH=c:\Espressif\tools\python\v5.5.2\venv && \
  set PATH=c:\Espressif\tools\python\v5.5.2\venv\Scripts;c:\Espressif\tools\cmake\3.30.2\bin;c:\Espressif\tools\ninja\1.12.1;c:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20251107\xtensa-esp-elf\bin;c:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin;%PATH% && \
  cd /d <YOUR_PROJECT_DIR>\build && \
  ninja > <YOUR_PROJECT_DIR>\build_log.txt 2>&1"

# 출력 읽기
cat build_log.txt
```

### 10.2 LVGL 구성 경고

**경고:** sdkconfig에서 `CONFIG_LV_TICK_PERIOD_MS`를 찾을 수 없음

**원인:** LVGL 9.x는 다른 구성 시스템 사용

**수정:** 경고 무시, 또는 `lv_conf.h`에서 구성

**실제 틱 속도:** `main/waveshare_display.cpp:90`에서 `LVGL_TICK_PERIOD_MS=16`으로 설정

## 11. 빌드 검증

빌드 후 확인사항:

- [ ] 오류 없이 빌드 완료
- [ ] 바이너리 크기 < 1 MB (factory 파티션)
- [ ] 메인 코드에 사용하지 않는 변수 경고 없음
- [ ] 플래시 성공
- [ ] 부팅 로그에 다음 항목 표시:
  - ESP-IDF 버전
  - 디스플레이 초기화 성공
  - LVGL 초기화됨
  - 섹터 타이밍 초기화됨
  - 모드 선택 (SIMULATION/GPS_HARDWARE)

## 12. CI/CD 고려사항

자동화된 빌드의 경우:

```yaml
# 예시 GitHub Actions
- name: Setup ESP-IDF
  uses: espressif/esp-idf-ci-action@v1
  with:
    esp_idf_version: v5.5.2
    target: esp32s3

- name: Build
  run: idf.py build

- name: Upload artifacts
  uses: actions/upload-artifact@v3
  with:
    name: firmware
    path: build/gps_laptimer.bin
```

## 13. 툴체인 버전

| 도구 | 버전 | 비고 |
|------|------|------|
| **CMake** | 3.30.2 | ESP-IDF 관리 |
| **Ninja** | 1.12.1 | 빌드 실행기 |
| **GCC** | 14.2.0 (esp-14.2.0_20251107) | Xtensa 크로스 컴파일러 |
| **Python** | 3.x | ESP-IDF venv |
| **esptool** | 4.x | 플래싱 도구 |

**버전 확인:**
```bash
idf.py --version
xtensa-esp32s3-elf-gcc --version
cmake --version
ninja --version
```

## 참조

- **ESP-IDF 문서:** https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/
- **하드웨어 사양:** [hardware.md](hardware.md)
- **아키텍처:** [architecture.md](architecture.md)
- **빠른 시작 가이드:** [../guides/quick-start.md](../guides/quick-start.md)
