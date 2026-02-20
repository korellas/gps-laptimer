# Phase 4-2: waveshare_display.cpp 분할 상세 계획

> **우선순위**: Low | **예상 변경량**: ~800줄 이동 + ~80줄 신규 헤더/접착 코드
> **전제조건**: Phase 4-1 (shim 헤더 정리) 완료 상태

---

## 1. 목표

`main/waveshare_display.cpp` (2,506줄)를 **HAL 레이어**와 **UI 레이어**로 분리한다.

| 파일 | 역할 | 예상 라인 |
|------|------|-----------|
| `main/display_hal.cpp` | LCD 패널, 터치, 백라이트, LVGL 인프라, I2C 버스, 전원 | ~780줄 |
| `main/display_hal.h` | HAL 공개 API (init, backlight, touch, mutex, power, I2C) | ~60줄 |
| `main/waveshare_display.cpp` | UI 위젯 생성/업데이트, 제스처, 알림, 페이지별 렌더링 | ~1,700줄 |

기존 `waveshare_display.h`는 UI 전용 API 헤더로 축소하고, `display_widgets.h`는 변경 없이 유지한다.

---

## 2. 현재 파일 구조 (라인 범위)

```
waveshare_display.cpp (2,506 lines)
├── L1-8      파일 헤더
├── L10-42    #include (HAL+UI 혼합)
├── L44-49    LV_FONT_DECLARE (UI)
├── L50       TAG 상수
├── L52-111   하드웨어 핀 #define + 디스플레이 설정 (HAL)
├── L113-129  HAL static 변수들 (panel, mutex, I2C, DMA 등)
├── L131-248  UI static 변수들 (위젯 포인터, 캐시, 델타 스무딩)
├── L255-370  lvgl_flush_cb()                          ← HAL
├── L372-384  lcd_trans_done_cb()                      ← HAL
├── L390-393  lvgl_tick_timer_cb()                     ← HAL
├── L399-437  터치 static + lvgl_touch_read_cb()       ← HAL
├── L443-452  lvgl_lock() / lvgl_unlock()              ← HAL
├── L458-473  lvgl_port_task()                         ← HAL
├── L475-496  init_backlight_from_example()            ← HAL
├── L499-509  setBacklight() / isBacklightOn()         ← HAL
├── L515-555  initPowerLatch()                         ← HAL
├── L557-677  init_lcd_hardware()                      ← HAL
├── L683-728  init_touch_hardware()                    ← HAL
├── L734-781  init_lvgl()                              ← HAL
├── L787-1116 setupUI()                                ← UI
├── L1122-1130 initDisplay()                           ← HAL
├── L1136-1526 updateLapData()                         ← UI
├── L1528-1537 updateGpsData() / updateTimeData()      ← UI
├── L1543-1553 GPS signal lost state                   ← UI
├── L1559-1590 notification statics + show/update      ← UI
├── L1596-1599 readTouch()                             ← HAL
├── L1601-1666 gesture detection                       ← UI
├── L1672-1807 createStartupScreen()                   ← UI
├── L1812-1815 getSensorI2CBus()                       ← HAL
├── L1817-1860 systemPowerOff()                        ← 혼합 (분리 필요)
├── L1862-1990 GPS status page update                  ← UI
├── L1996-2003 resetDeltaHistory()                     ← UI
├── L2009-2054 displayTest()                           ← UI
├── L2060-2061 lvglLock() / lvglUnlock() (공개 래퍼)   ← HAL
├── L2063-2069 pollGesture/updateNotificationDisplay   ← UI
├── L2071-2102 updateBatteryWarning()                  ← UI
├── L2104-2142 위젯 getter 함수들                      ← UI
├── L2148-2244 applyPageVisibilityForPage()            ← UI
├── L2246-2258 GPS status helpers                      ← UI
├── L2260-2271 updatePreTrackDisplay()                 ← UI
├── L2276-2389 updateLapSummaryDisplay()               ← UI
├── L2391-2406 laptimer display 래퍼들                  ← UI
└── L2412-2505 updateImuDisplay()                      ← UI
```

---

## 3. 핵심 커플링 포인트 (Critical Shared State)

분할 시 가장 주의해야 할 공유 상태:

### 3.1 `lvgl_mutex` (L119)
- **소유자**: HAL — `init_lvgl()`에서 생성
- **사용처**: 모든 UI 함수가 `xSemaphoreTake(lvgl_mutex, ...)` 호출
- **해결**: `display_hal.h`에서 `lvglLock()`/`lvglUnlock()` 공개 API 제공 (이미 존재)
- **주의**: UI 코드에서 `lvgl_mutex` 직접 접근을 모두 `lvglLock()`/`lvglUnlock()` 호출로 교체

### 3.2 `s_touchPressed` / `s_touchX` / `s_touchY` (L400-402)
- **소유자**: HAL — `lvgl_touch_read_cb()`가 LVGL 입력 콜백 문맥(전용 LVGL task)에서 쓰기
- **사용처**: UI의 `readTouch()`, `detectGesture()`가 읽기
- **해결**: `readTouch()`를 HAL에 유지. `detectGesture()`에서 필요한 터치 좌표는
  `readTouchXY(int16_t* x, int16_t* y)` HAL API 추가
- **동기화 포인트**: ISR 안전성 이슈가 아니라 HAL/UI 경계에서
  `readTouch()`/`readTouchXY()` 계약을 단일화하는 것이 핵심

### 3.3 `s_sensor_bus` / `s_io_expander` (L126-127)
- **소유자**: HAL 전용
- **사용처**: `getSensorI2CBus()` (이미 HAL), `systemPowerOff()` (혼합 함수)
- **해결**: `systemPowerOff()`를 HAL에 배치 (전원 제어가 본질적 역할)

### 3.4 `panel_handle` (L117)
- **소유자**: HAL 전용
- **사용처**: HAL의 `lvgl_flush_cb()`, `init_lcd_hardware()`, `systemPowerOff()`
- **해결**: HAL 내부에서만 사용, 문제 없음

### 3.5 Frame 데이터 (`tframe`, `gframe`, `lframe`, 유효 플래그) (L213-219)
- **현 위치**: `waveshare_display.cpp`에 정의, `waveshare_display.h`에서 extern
- **사용처**: UI 코드에서만 읽기 (`updateLapData()`, `updateTimeData()`, `updateGpsData()`)
- **해결**: UI 레이어(`waveshare_display.cpp`)에 그대로 유지

### 3.6 `LCD_H_RES` / `LCD_V_RES` 매크로 (L92-93)
- **사용처**: HAL (`lvgl_flush_cb`, `init_lvgl`) + UI (`setupUI`, `systemPowerOff`, `createStartupScreen` 등)
- **해결**: `display_hal.h`에 정의, UI에서 include

---

## 4. 구현 상세

### 4.1 `display_hal.h` (신규 파일)

```cpp
/**
 * @file display_hal.h
 * @brief Hardware Abstraction Layer for Waveshare ESP32-S3-Touch-LCD-3.49
 *
 * LCD panel, touch, backlight, LVGL infrastructure, I2C bus, power control.
 * 기본 전략: waveshare_display.h 경유 include로 하위호환 유지,
 * HAL-only 콜사이트는 필요 시 direct include로 점진 전환.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Forward declaration
typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

// ── Display resolution (after rotation to landscape) ──
#define LCD_H_RES  640
#define LCD_V_RES  172

// ── Initialization (call in order) ──
void initPowerLatch(void);
void initDisplay(void);

// ── LVGL mutex (for UI thread safety) ──
bool lvglLock(int timeoutMs);  // timeoutMs < 0 => portMAX_DELAY
void lvglUnlock(void);

// ── Backlight ──
void setBacklight(bool on);
bool isBacklightOn(void);

// ── Touch ──
bool readTouch(void);
bool readTouchXY(int16_t* x, int16_t* y);  // NEW: raw coords for gesture
// Contract:
// - x == NULL 또는 y == NULL 이면 false 반환 (출력 쓰기 없음)
// - 터치가 없으면 false 반환 + *x = 0, *y = 0

// ── I2C sensor bus (shared GPIO47/48: TCA9554, RTC, IMU) ──
i2c_master_bus_handle_t getSensorI2CBus(void);

// ── Power ──
void systemPowerOff(void);

#ifdef __cplusplus
}
#endif
```

### 4.2 `display_hal.cpp` (신규 파일)

아래 영역을 `waveshare_display.cpp`에서 **이동**:

```
이동할 코드 블록 (순서대로):
─────────────────────────────────
1. 파일 헤더 (신규 작성)
2. HAL용 #include 블록:
   - freertos/FreeRTOS.h, freertos/task.h, freertos/semphr.h
   - esp_log.h, esp_timer.h
   - esp_lcd_panel_io.h, esp_lcd_panel_ops.h, esp_lcd_axs15231b.h
   - driver/gpio.h, driver/spi_master.h, driver/i2c_master.h, driver/ledc.h
   - lvgl.h
   - esp_io_expander_tca9554.h, esp_heap_caps.h
   - "display_hal.h"  (자체 헤더)
   - "config.h"       (DISPLAY_ROTATION_DEG 참조)
   ※ systemPowerOff()가 LVGL UI 코드를 포함하므로 lvgl.h가 HAL에서도 필요
   ※ share_tech_mono_32 font 선언:
        LV_FONT_DECLARE(share_tech_mono_32);   ← display_hal.cpp 상단에 추가

3. TAG 상수:                          static const char* TAG = "DISPLAY";
4. 핀 #define 블록:                   L52-106 전체
5. 회전 검증 + constexpr:             L107-111 전체
   - static_assert(DISPLAY_ROTATION_DEG == 90 || DISPLAY_ROTATION_DEG == 270, ...)
   - static constexpr lv_display_rotation_t kDisplayRotation = ...
   ※ kDisplayRotation은 init_lvgl()에서만 사용하므로 HAL 영역 맞음
6. HAL static 변수들:                 L113-129 전체
   - panel_handle, lvgl_disp, lvgl_mutex, s_flush_done_semaphore
   - s_logged_first_flush/flush_error/rotated_flush
   - s_touch_bus, s_touch_dev
   - s_sensor_bus, s_io_expander
   - s_lvgl_rot_buf, s_lvgl_dma_buf
7. 터치 volatile 변수:               L399-402
   - s_touchPressed, s_touchX, s_touchY
8. 백라이트 상태:                      L499 (s_backlightOn)
9. lvgl_flush_cb():                   L255-370
10. lcd_trans_done_cb():               L372-384
11. lvgl_tick_timer_cb():              L390-393
12. lvgl_touch_read_cb():              L404-437
13. lvgl_lock() / lvgl_unlock():       L443-452
14. lvgl_port_task():                  L458-473
15. init_backlight_from_example():     L475-496
16. setBacklight() / isBacklightOn():  L499-509
17. initPowerLatch():                  L515-555
18. init_lcd_hardware():               L557-677
19. init_touch_hardware():             L683-728
20. init_lvgl():                       L734-781
21. initDisplay():                     L1122-1130
22. readTouch():                       L1596-1599
23. readTouchXY() (신규):              s_touchPressed/X/Y 반환
24. getSensorI2CBus():                 L1812-1815
25. systemPowerOff():                  L1817-1860
26. lvglLock() / lvglUnlock() 래퍼:    L2060-2061
```

### 4.3 `waveshare_display.cpp` 수정

이동 후 남는 코드:

```
남는 코드 블록 (순서대로):
─────────────────────────────────
1. 파일 헤더 (수정: "UI layer" 명시)
2. UI용 #include 블록:
   - "waveshare_display.h"   (자체 공개 헤더)
   - "display_hal.h"         (신규: lvglLock, LCD_H_RES 등)
   - "display_widgets.h"
   - "config.h", "types.h", "protocol.hpp"
   - "sector_timing.h", "ublox_gps.h", "display_config.h", "sensor_fusion.h"
   - <cstring>, <cstdio>, <cmath>, <ctime>
   - "freertos/FreeRTOS.h", "freertos/task.h"
   - "esp_log.h", "esp_timer.h"
   - "lvgl.h"
   ※ 제거: esp_lcd_*, driver/*, esp_io_expander_*, esp_heap_caps.h
   ※ 제거: ble_ota.h, wifi_portal.h, esp_wifi.h (미사용)

3. LV_FONT_DECLARE 4개:              share_tech_mono_24/32/56/72
4. TAG 상수:                          static const char* TAG = "DISPLAY_UI";
5. UI 위젯 포인터들:                  L131-211 전체
6. Frame 데이터 (extern):             L213-219
7. 캐시 구조체 + 델타 스무딩:         L221-248
8. setupUI():                         L787-1116
9. updateLapData():                   L1136-1526
10. updateGpsData() / updateTimeData(): L1528-1537
11. GPS signal lost state:             L1543-1553
12. notification statics + 함수들:     L1559-1590
13. detectGesture():                   L1601-1666
    ※ 수정: s_touchX/Y 직접 접근 → readTouchXY() 호출로 교체
14. createStartupScreen():             L1672-1807
15. updateGpsStatusPage():             L1862-1990
16. resetDeltaHistory():               L1996-2003
17. displayTest():                     L2009-2054
18. pollGesture():                     L2063-2067
19. updateNotificationDisplay():       L2069
20. updateBatteryWarning():            L2071-2102
21. 위젯 getter 함수들:               L2104-2142
22. applyPageVisibilityForPage():      L2148-2244
23. GPS status helpers:                L2246-2258
24. updatePreTrackDisplay():           L2260-2271
25. updateLapSummaryDisplay():         L2276-2389
26. laptimer 래퍼들:                   L2391-2406
27. updateImuDisplay():                L2412-2505
```

### 4.4 `components/common/include/waveshare_display.h` 수정

**중요**: `waveshare_display.h`는 `components/common/include/`에 위치하며,
`display_hal.h`는 `main/`에 위치한다. 컴포넌트 빌드 시 `main/` 경로는 include
검색 대상이 아니므로, **`waveshare_display.h`는 `display_hal.h`를 include할 수 없다**.
→ HAL API 선언은 waveshare_display.h에서 단순 삭제한다 (재노출 없음).

```diff
  #include "protocol.hpp"
  // display_hal.h는 포함하지 않음 (main/ 헤더이므로 컴포넌트에서 접근 불가)

  // Frame data (유지)
  extern struct tFrame tframe;
  ...

- // Early power latch
- void initPowerLatch(void);
- // Initialization
- void initDisplay(void);
+ // Initialization (UI layer — call after initDisplay())
  void setupUI(void);

  // GPS signal status (유지)
  void setGpsSignalLost(bool lost);
  bool isGpsSignalLost(void);

  // Notifications (유지)
  void showNotification(const char* message, uint16_t durationMs);

  // Data updates (유지)
  void updateLapData(void);
  void updateGpsData(void);
  void updateTimeData(void);

- // Touch handling
- bool readTouch(void);
-
  // Delta history reset (유지)
  void resetDeltaHistory(void);

  // Startup screen (유지)
  void createStartupScreen(void);

- // Sensor I2C bus
- i2c_master_bus_handle_t getSensorI2CBus(void);
-
- // Backlight control
- void setBacklight(bool on);
- bool isBacklightOn(void);
-
- // Power management
- void systemPowerOff(void);

  // Display test (유지)
  void displayTest(void);
```

경로/계층 경계 메모 (현재 코드 snapshot 기준):
- 공개 UI 헤더 경로: `components/common/include/waveshare_display.h`
  - 컴포넌트에서 include 가능 (gps_processor.cpp, simulation.cpp 등)
  - HAL API 선언 완전 제거 → UI API만 유지
- `display_hal.h`는 `main/` 레이어 경계 헤더로 유지
  - `components/*`는 include 불가 (main/ 경로 미노출)
  - `main/` 내 파일들만 직접 include

### 4.5 `main/display_widgets.h` 수정

`display_widgets.h`(main/)는 `lvglLock()/lvglUnlock()`을 직접 선언하고 있는데,
`display_hal.h`(main/)도 동일 함수를 선언할 예정이다. 두 헤더를 동시에 include하면
중복 선언이 된다(C++에서는 legal이지만 불필요). 해결책: `display_widgets.h`에서
중복 선언을 제거하고 `display_hal.h`를 include한다.

```diff
  #pragma once

  #include "lvgl.h"
  #include "page.h"
+ #include "display_hal.h"   // lvglLock, lvglUnlock, LCD_H_RES, LCD_V_RES

- // ── LVGL mutex helpers ──
- bool lvglLock(int timeoutMs);
- void lvglUnlock();

  // ── Gesture polling ──
  ...
```

`display_widgets.h`를 include하는 모든 파일 (~14개: pages/*.cpp, page_manager.cpp 등)이
`display_hal.h`의 선언을 자동으로 상속받으므로, 개별 파일에 display_hal.h를 추가할 필요 없다.

### 4.6 `lvgl_mutex` 직접 접근 제거

UI 코드에서 `lvgl_mutex`를 직접 사용하는 곳을 모두 `lvglLock()`/`lvglUnlock()`으로 교체:

| 함수 | 현재 패턴 | 변경 후 |
|------|-----------|---------|
| `setupUI()` | `xSemaphoreTake(lvgl_mutex, portMAX_DELAY)` | `lvglLock(-1)` |
| `updateLapData()` | `lvgl_lock(50)` 내부 static 호출 | 이미 OK — `lvgl_lock` static이 HAL로 이동하므로 `lvglLock(50)` 공개 함수 사용 |
| `updateGpsStatusPage()` | `xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50))` | `lvglLock(50)` |
| `showNotification()` | `xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(100))` | `lvglLock(100)` |
| `updateNotification()` | `xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(30))` | `lvglLock(30)` |
| `updateBatteryWarning()` | `lvgl_lock(50)` / `lvgl_unlock()` | `lvglLock(50)` / `lvglUnlock()` |
| `applyPageVisibilityForPage()` | `xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(100))` | `lvglLock(100)` |
| `updateLapSummaryDisplay()` | `xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50))` | `lvglLock(50)` |
| `updateImuDisplay()` | `lvglLock(50)` / `lvglUnlock()` | 이미 OK |
| `createStartupScreen()` | `xSemaphoreTake(lvgl_mutex, portMAX_DELAY)` | `lvglLock(-1)` |
| `displayTest()` | `xSemaphoreTake(lvgl_mutex, portMAX_DELAY)` | `lvglLock(-1)` |

**요약**: 약 10곳에서 `xSemaphoreTake(lvgl_mutex, ...)` → `lvglLock(ms)`, `xSemaphoreGive(lvgl_mutex)` → `lvglUnlock()` 교체

### 4.7 `detectGesture()` 터치 접근 수정

```cpp
// 변경 전 (s_touchPressed/X/Y 직접 접근):
static GestureResult detectGesture() {
    bool pressed = s_touchPressed;
    int16_t x = s_touchX;
    int16_t y = s_touchY;
    ...
}

// 변경 후 (HAL API 통해 접근):
static GestureResult detectGesture() {
    int16_t x, y;
    bool pressed = readTouchXY(&x, &y);
    ...
}
```

### 4.8 `systemPowerOff()` 분리 전략

`systemPowerOff()`는 UI(종료 메시지 오버레이)와 HAL(IO expander 전원 차단)이 혼합되어 있다.

**결정**: HAL에 배치 (전원 제어가 핵심 역할)
- LVGL 위젯 생성 부분은 font declaration만 HAL 쪽에서 접근 필요
- `share_tech_mono_32` font은 `LV_FONT_DECLARE`로 HAL에서도 선언 가능
- `LCD_H_RES`/`LCD_V_RES`는 이미 `display_hal.h`에 정의
- 종료 오버레이 표시 실패 시에도 전원 차단 시퀀스는 계속 진행 (Fail-Open for UI)
- `lvgl_mutex` 미초기화(NULL) 또는 `lvglLock()` 획득 실패 시
  UI 표시 단계를 건너뛰고 전원 차단 단계로 즉시 진행

### 4.9 `main/CMakeLists.txt` 수정

```diff
  SRCS
      "main.cpp"
+     "display_hal.cpp"
      "waveshare_display.cpp"
      ...
```

---

## 5. 수정이 필요한 외부 파일

`display_hal.h`로 이동한 API를 사용하는 파일들 (현재 코드 snapshot 기준):

| 파일 | 사용하는 HAL API | 변경 (현재 코드 snapshot 기준) |
|------|-----------------|------|
| `main/main.cpp` | `initPowerLatch()`, `initDisplay()`, `getSensorI2CBus()`, `systemPowerOff()` | `#include "display_hal.h"` 추가 |
| `main/page_manager.cpp` | `readTouch()`, `setBacklight()`, `systemPowerOff()` | `#include "display_hal.h"` 추가 |
| `components/modes/gps_processor.cpp` | `setGpsSignalLost()` → 이건 UI이므로 waveshare_display.h에 유지 | 변경 없음 |
| `main/pages/wait_gps_page.cpp` | `readTouch()` | `#include "display_hal.h"` 추가 (또는 `waveshare_display.h` 경유) |
| `main/serial_commands.cpp` | `systemPowerOff()` | `#include "display_hal.h"` 추가 (또는 `waveshare_display.h` 경유) |

**핵심**: 기본 전략은 하위호환 유지다. `waveshare_display.h`가 `display_hal.h`를 include해서
기존 include 체인을 깨지 않는다.
HAL API만 쓰는 파일의 direct include 전환은 빌드 안정화 후 점진 정리(선택)로 진행한다.

**추천**: 일괄 교체 대신, `waveshare_display.h`의 `display_hal.h` 재노출을 유지하고
외부 파일 include 정리는 선택적으로 단계 진행

---

## 6. 실행 순서

```
Step 1: display_hal.h 생성 (main/display_hal.h)
Step 2: display_hal.cpp 생성 (waveshare_display.cpp에서 HAL 영역 이동)
        - LV_FONT_DECLARE(share_tech_mono_32) 추가 (systemPowerOff용)
        - kDisplayRotation constexpr + static_assert 포함 이동
Step 3: waveshare_display.cpp 정리
        - 이동된 코드 삭제
        - #include 정리 (HAL 드라이버 헤더 제거, display_hal.h 추가)
        - lvgl_mutex 직접 접근 → lvglLock/Unlock 교체 (~10곳)
        - detectGesture() → readTouchXY() 사용
Step 4: components/common/include/waveshare_display.h 수정
        - HAL API 선언 제거 (display_hal.h include 없이 단순 삭제)
Step 5: display_widgets.h 수정
        - lvglLock/lvglUnlock 중복 선언 제거
        - #include "display_hal.h" 추가
Step 6: main/CMakeLists.txt에 display_hal.cpp 추가
Step 7: 외부 파일 #include 업데이트
        - main/main.cpp: display_hal.h 추가 (initPowerLatch, initDisplay 등)
        - main/page_manager.cpp: display_hal.h 추가 (readTouch, setBacklight 등)
        - main/pages/wait_gps_page.cpp: display_hal.h 추가 (readTouch)
        - main/serial_commands.cpp: display_hal.h 추가 (systemPowerOff)
Step 8: 빌드 검증 (idf.py build)
Step 9: 커밋
```

---

## 7. 검증 체크리스트

- [ ] `idf.py build` 성공
- [ ] `display_hal.cpp`에 UI 관련 코드 없음 (lv_label_create 등은 systemPowerOff만 예외)
- [ ] `waveshare_display.cpp`에 드라이버 코드 없음 (esp_lcd_*, i2c_master_* 등)
- [ ] `lvgl_mutex`가 UI 코드에서 직접 참조되지 않음
- [ ] `s_touchPressed/X/Y`가 UI 코드에서 직접 참조되지 않음
- [ ] `readTouchXY(NULL, &y)` / `readTouchXY(&x, NULL)`가 false 반환하고 출력 쓰기를 하지 않음
- [ ] `readTouchXY(&x, &y)`에서 no-touch 시 false 반환 + `(0,0)` 보정 규약 준수
- [ ] `systemPowerOff()`에서 `lvgl_mutex` 미초기화 또는 lock 실패 시에도 전원 차단 시퀀스 진행
- [ ] `rg` 기준으로 섹션 5 외부 파일 표와 실제 HAL API 콜사이트가 일치
- [ ] flash → 전체 기능 테스트 (LCD 표시, 터치, 백라이트, 전원 Off)

---

## 8. 리스크 및 주의사항

1. **`systemPowerOff()` 내 LVGL 위젯**: HAL에 배치하되 `lvgl.h`와 font를 HAL에서도 include해야 함.
   종료 오버레이 표시가 실패해도 전원 차단 시퀀스는 계속 진행해야 함(정책 고정).

2. **`lvglLock()` 시그니처**: `bool lvglLock(int timeoutMs)`에서 `timeoutMs < 0`은
   `portMAX_DELAY`로 해석하도록 계약을 고정. 무기한 대기 호출은 `lvglLock(-1)`로 통일.

3. **`readTouchXY()` 계약 불일치 위험**: 구현/호출부 중 한쪽이라도 false 경로에서 좌표 처리
   규약(`(0,0)` 보정, NULL 입력 false)을 어기면 제스처 오검출 가능.

4. **컴파일 순서**: `display_hal.cpp`가 `config.h`의 `DISPLAY_ROTATION_DEG`를 참조하므로
   `common` 컴포넌트 의존성 유지 필요 (이미 `main/CMakeLists.txt` REQUIRES에 포함).

5. **include 경로 계층 경계**: `waveshare_display.h`(components/common/include/)는
   `display_hal.h`(main/)를 include할 수 없다. components/* 빌드 시 main/ 경로가
   include 탐색 대상이 아니기 때문. 이 경계를 어기면 컴포넌트 빌드 실패.
   → 해결: waveshare_display.h는 HAL API를 재노출하지 않고 단순 삭제.
     HAL이 필요한 main/ 파일들은 display_hal.h를 직접 include.

6. **바이너리 크기**: 코드 이동만이므로 바이너리 크기 변화 없음 (±수백 바이트 이내).
