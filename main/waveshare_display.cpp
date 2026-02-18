/**
 * @file waveshare_display.cpp
 * @brief Display driver for Waveshare ESP32-S3-Touch-LCD-3.49 (ESP-IDF + LVGL)
 * @version 3.0 - ESP-IDF native implementation
 *
 * Hardware: AXS15231B LCD (QSPI), Touch (I2C)
 * Resolution: 172x640 native, rotated to 640x172 landscape
 */

#include "waveshare_display.h"
#include "display_widgets.h"
#include "config.h"
#include "types.h"
#include "protocol.hpp"
#include "sector_timing.h"
#include "ublox_gps.h"
#include "ble_ota.h"
#include "wifi_portal.h"
#include "display_config.h"

#include <cstring>
#include <cstdio>
#include <cmath>
#include <ctime>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_axs15231b.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "lvgl.h"
#include "esp_io_expander_tca9554.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"

// Share Tech Mono font declarations
LV_FONT_DECLARE(share_tech_mono_24);
LV_FONT_DECLARE(share_tech_mono_32);
LV_FONT_DECLARE(share_tech_mono_56);

static const char *TAG = "DISPLAY";

// ============================================================
// HARDWARE PINS - Waveshare ESP32-S3-Touch-LCD-3.49
// ============================================================

// LCD SPI Pins (QSPI mode) - Waveshare 3.49 reference
#define LCD_SPI_HOST    SPI3_HOST
#define LCD_PIN_SCLK    GPIO_NUM_10
#define LCD_PIN_DATA0   GPIO_NUM_11
#define LCD_PIN_DATA1   GPIO_NUM_12
#define LCD_PIN_DATA2   GPIO_NUM_13
#define LCD_PIN_DATA3   GPIO_NUM_14
#define LCD_PIN_CS      GPIO_NUM_9
#define LCD_PIN_RST     GPIO_NUM_21
#define LCD_PIN_BL      GPIO_NUM_8

// Backlight PWM values follow Waveshare examples (inverted duty)
#define LCD_PWM_MODE_0   (0xff - 0)
#define LCD_PWM_MODE_255 (0xff - 255)

// Touch I2C Pins (CST816S)
#define TOUCH_I2C_PORT  I2C_NUM_0
#define TOUCH_PIN_SDA   GPIO_NUM_17
#define TOUCH_PIN_SCL   GPIO_NUM_18
#define TOUCH_PIN_RST   (-1)
#define TOUCH_PIN_INT   (-1)
#define CST816S_ADDR    0x3b

// IO expander (power/enable) - Waveshare example uses SDA/SCL on 47/48
#define IOEXP_I2C_PORT  I2C_NUM_1
#define IOEXP_PIN_SDA   GPIO_NUM_47
#define IOEXP_PIN_SCL   GPIO_NUM_48

// ============================================================
// DISPLAY CONFIGURATION
// ============================================================

// Native resolution (before rotation)
#define LCD_NATIVE_H    172
#define LCD_NATIVE_V    640
// Logical resolution after rotation (landscape)
#define LCD_H_RES       640
#define LCD_V_RES       172
#define LCD_DMA_LINES   64
#define LCD_FULL_BUF_SIZE (LCD_NATIVE_H * LCD_NATIVE_V * sizeof(lv_color_t))
#define LCD_DMA_BUF_SIZE  (LCD_NATIVE_H * LCD_DMA_LINES * sizeof(uint16_t))

// LVGL tick period (matches Waveshare example: 5ms)
#define LVGL_TICK_PERIOD_MS 5

// LVGL dedicated task parameters (matches Waveshare example)
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 10
#define LVGL_TASK_STACK_SIZE   (8 * 1024)
#define LVGL_TASK_PRIORITY     2

static_assert(DISPLAY_ROTATION_DEG == 90 || DISPLAY_ROTATION_DEG == 270,
              "DISPLAY_ROTATION_DEG must be 90 or 270 for 640x172 landscape UI");

static constexpr lv_display_rotation_t kDisplayRotation =
    (DISPLAY_ROTATION_DEG == 90) ? LV_DISPLAY_ROTATION_90 : LV_DISPLAY_ROTATION_270;

// ============================================================
// GLOBAL OBJECTS
// ============================================================

static esp_lcd_panel_handle_t panel_handle = NULL;
static lv_display_t *lvgl_disp = NULL;
static SemaphoreHandle_t lvgl_mutex = NULL;
static SemaphoreHandle_t s_flush_done_semaphore = NULL;
static bool s_logged_first_flush = false;
static bool s_logged_flush_error = false;
static bool s_logged_rotated_flush = false;
static i2c_master_bus_handle_t s_touch_bus = nullptr;
static i2c_master_dev_handle_t s_touch_dev = nullptr;
static i2c_master_bus_handle_t s_sensor_bus = nullptr;
static esp_io_expander_handle_t s_io_expander = nullptr;
static uint8_t *s_lvgl_rot_buf = nullptr;
static uint16_t *s_lvgl_dma_buf = nullptr;

// LVGL UI elements
static lv_obj_t *lbl_laptime = NULL;
static lv_obj_t *lbl_lapnum = NULL;
static lv_obj_t *lbl_best = NULL;
static lv_obj_t *lbl_delta = NULL;
static lv_obj_t *lbl_speed_delta = NULL;
static lv_obj_t *bar_bg = NULL;
static lv_obj_t *bar_up = NULL;
static lv_obj_t *bar_down = NULL;
static lv_obj_t *lbl_notification = NULL;

// 저전력 경고 텍스트
static lv_obj_t *lbl_bat_low = NULL;          // main screen
static lv_obj_t *lbl_bat_low_startup = NULL;  // startup overlay
static lv_obj_t *lbl_bat_low_phone = NULL;    // phone overlay

// 날짜/시간 라벨 (delta 위, 중앙)
static lv_obj_t *lbl_datetime = NULL;

// Sector delta labels (displayed under lap time)
static lv_obj_t *lbl_sector_deltas[3] = {NULL, NULL, NULL};  // S1, S2, S3

// GPS Status page labels
static constexpr int GPS_STATUS_LINE_COUNT = 5;
static constexpr int GPS_STATUS_LINE_WIDTH = 96;
static lv_obj_t *gps_status_labels[GPS_STATUS_LINE_COUNT] = {};

// Lap completion fullscreen overlay
static lv_obj_t *lap_complete_overlay = NULL;
static lv_obj_t *lap_complete_time_label = NULL;
static lv_obj_t *lap_complete_border = NULL;

// Startup screen overlay (화면: STARTUP)
static lv_obj_t *startup_overlay = NULL;
static lv_obj_t *startup_title_label = NULL;
static lv_obj_t *startup_version_label = NULL;
static lv_obj_t *startup_status_label = NULL;
static lv_obj_t *startup_hint_label = NULL;

// Phone plate overlay (화면: PHONE_PLATE)
static lv_obj_t *phone_overlay = NULL;
static lv_obj_t *phone_number_label = NULL;
static lv_obj_t *phone_hint_label = NULL;

// PRE_TRACK 모드 상태 (display helper 함수에서 사용)
static bool s_preTrackMode = false;
static char s_preTrackName[64] = "";
static int s_userPageIndex = 0;  // index into getDisplayConfig().pages[]

// Frame data
struct tFrame tframe = {};
struct gFrame gframe = {};
struct lFrame lframe = {};
bool gvalid = false;
bool tvalid = false;
bool lvalid = false;

// ============================================================
// CACHE FOR SMART UPDATES
// ============================================================

static struct {
    char lapTime[16] = "";
    char lapNum[16] = "";
    char best[64] = "";  // Increased for multi-line top 3 laps
    char delta[16] = "";
    char speedDelta[24] = "";
    char time[8] = "";
    int32_t lastBarValue = -9999;
    bool lastBarPositive = false;
    int8_t lastLowBat = -1;    // 저전력 글로우 상태 (-1=미설정, 0=off, 1=on)
} cache;

// ============================================================
// DELTA SMOOTHING
// ============================================================

static float smoothedDelta = 0.0f;
static float smoothedSpeedDeltaKmh = 0.0f;

static const float DELTA_EMA_ALPHA = 0.15f;
static const float SPEED_DELTA_EMA_ALPHA = 0.20f;
static const float SPEED_BAR_RANGE_KMH = 10.0f;
static const float SPEED_BAR_DEADZONE_KMH = 0.25f;


// ============================================================
// LVGL FLUSH CALLBACK
// ============================================================

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (!s_logged_first_flush) {
        ESP_LOGI(TAG, "LVGL flush first area: x1=%d y1=%d x2=%d y2=%d", area->x1, area->y1, area->x2, area->y2);
        s_logged_first_flush = true;
    }
    // Match Waveshare example: full-frame render + software rotation + fixed-size DMA chunk uploads.
    bool full_frame = (area->x1 == 0 && area->y1 == 0 &&
                       area->x2 == (LCD_H_RES - 1) && area->y2 == (LCD_V_RES - 1));
    if (!full_frame && !s_logged_flush_error) {
        ESP_LOGW(TAG, "Non-full flush area received, using fallback rotate path");
    }

    lv_color_format_t cf = lv_display_get_color_format(disp);
    lv_area_t draw_area = {0, 0, LCD_NATIVE_H - 1, LCD_NATIVE_V - 1};
    uint8_t *draw_map = s_lvgl_rot_buf;
    lv_draw_sw_rgb565_swap(px_map, (uint32_t)lv_area_get_size(area));

    if (full_frame) {
        lv_display_rotation_t rotation = lv_display_get_rotation(disp);
        if (rotation != LV_DISPLAY_ROTATION_0) {
            uint32_t src_stride = lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
            uint32_t dst_stride = lv_draw_buf_width_to_stride(LCD_NATIVE_H, cf);
            lv_draw_sw_rotate(px_map, s_lvgl_rot_buf,
                              lv_area_get_width(area), lv_area_get_height(area),
                              src_stride, dst_stride, rotation, cf);
            draw_map = s_lvgl_rot_buf;
        } else {
            draw_map = px_map;
            draw_area = *area;
        }
    } else {
        lv_area_t rotated_area = *area;
        lv_display_rotation_t rotation = lv_display_get_rotation(disp);
        if (rotation != LV_DISPLAY_ROTATION_0) {
            lv_display_rotate_area(disp, &rotated_area);
        }
        uint32_t src_stride = lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
        uint32_t dst_stride = lv_draw_buf_width_to_stride(lv_area_get_width(&rotated_area), cf);
        lv_draw_sw_rotate(px_map, s_lvgl_rot_buf,
                          lv_area_get_width(area), lv_area_get_height(area),
                          src_stride, dst_stride, rotation, cf);
        draw_area = rotated_area;
        draw_map = s_lvgl_rot_buf;
    }

    if (!s_logged_rotated_flush) {
        ESP_LOGI(TAG, "LVGL rotated flush area: x1=%d y1=%d x2=%d y2=%d",
                 draw_area.x1, draw_area.y1, draw_area.x2, draw_area.y2);
        s_logged_rotated_flush = true;
    }

    esp_err_t err = ESP_OK;

    if (!s_flush_done_semaphore) {
        ESP_LOGE(TAG, "Flush semaphore is null");
        lv_display_flush_ready(disp);
        return;
    }

    xSemaphoreGive(s_flush_done_semaphore);
    if (full_frame) {
        const int flush_count = LCD_NATIVE_V / LCD_DMA_LINES; // 640/64 = 10
        const int offgap = LCD_NATIVE_V / flush_count;
        const int dmalen = LCD_DMA_BUF_SIZE / 2;              // uint16_t words per DMA chunk
        int offsetx1 = 0;
        int offsety1 = 0;
        int offsetx2 = LCD_NATIVE_H;
        int offsety2 = offgap;

        const uint16_t *map = reinterpret_cast<const uint16_t *>(draw_map);
        for (int i = 0; i < flush_count && err == ESP_OK; i++) {
            if (xSemaphoreTake(s_flush_done_semaphore, pdMS_TO_TICKS(50)) != pdTRUE) {
                err = ESP_ERR_TIMEOUT;
                break;
            }
            memcpy(s_lvgl_dma_buf, map, LCD_DMA_BUF_SIZE);
            err = esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2, offsety2, s_lvgl_dma_buf);
            offsety1 += offgap;
            offsety2 += offgap;
            map += dmalen;
        }
    } else {
        int32_t area_w = lv_area_get_width(&draw_area);
        const uint16_t *src = reinterpret_cast<const uint16_t *>(draw_map);
        int32_t y = draw_area.y1;
        int32_t y_end = draw_area.y2 + 1;
        while (y < y_end && err == ESP_OK) {
            int32_t lines = y_end - y;
            if (lines > LCD_DMA_LINES) lines = LCD_DMA_LINES;
            if (xSemaphoreTake(s_flush_done_semaphore, pdMS_TO_TICKS(50)) != pdTRUE) {
                err = ESP_ERR_TIMEOUT;
                break;
            }
            size_t px_count = (size_t)area_w * (size_t)lines;
            memcpy(s_lvgl_dma_buf, src, px_count * sizeof(uint16_t));
            err = esp_lcd_panel_draw_bitmap(panel_handle,
                                            draw_area.x1, y,
                                            draw_area.x1 + area_w, y + lines,
                                            s_lvgl_dma_buf);
            src += px_count;
            y += lines;
        }
    }

    if (err == ESP_OK && xSemaphoreTake(s_flush_done_semaphore, pdMS_TO_TICKS(50)) != pdTRUE) {
        err = ESP_ERR_TIMEOUT;
    }

    if (err != ESP_OK && !s_logged_flush_error) {
        ESP_LOGE(TAG, "LVGL flush draw failed: %s", esp_err_to_name(err));
        s_logged_flush_error = true;
    }

    lv_display_flush_ready(disp);
}

static bool lcd_trans_done_cb(esp_lcd_panel_io_handle_t panel_io,
                              esp_lcd_panel_io_event_data_t *edata,
                              void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    BaseType_t high_task_awoken = pdFALSE;
    SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(user_ctx);
    if (sem) {
        xSemaphoreGiveFromISR(sem, &high_task_awoken);
    }
    return high_task_awoken == pdTRUE;
}

// ============================================================
// LVGL TICK TIMER
// ============================================================

static void lvgl_tick_timer_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// ============================================================
// LVGL TOUCH INPUT (matches Waveshare example protocol)
// ============================================================

// 공유 터치 상태 (LVGL 콜백에서 기록, 다른 모듈에서 읽기)
static volatile bool s_touchPressed = false;
static volatile int16_t s_touchX = 0;
static volatile int16_t s_touchY = 0;

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    if (!s_touch_dev) {
        data->state = LV_INDEV_STATE_RELEASED;
        s_touchPressed = false;
        return;
    }
    uint8_t cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x0e, 0x0, 0x0, 0x0};
    uint8_t buf[32] = {0};
    esp_err_t ret = i2c_master_transmit_receive(s_touch_dev, cmd, 11, buf, 32, pdMS_TO_TICKS(20));
    if (ret != ESP_OK) {
        // 타임아웃 시 마지막 유효 상태 유지
        data->state = s_touchPressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        return;
    }
    if (buf[1] > 0 && buf[1] < 5) {
        uint16_t pointX = (((uint16_t)buf[2] & 0x0f) << 8) | (uint16_t)buf[3];
        uint16_t pointY = (((uint16_t)buf[4] & 0x0f) << 8) | (uint16_t)buf[5];
        if (pointX > LCD_NATIVE_V) pointX = LCD_NATIVE_V;
        if (pointY > LCD_NATIVE_H) pointY = LCD_NATIVE_H;
        int16_t lx = (int16_t)pointY;
        int16_t ly = (int16_t)(LCD_NATIVE_V - pointX);
        data->point.x = lx;
        data->point.y = ly;
        data->state = LV_INDEV_STATE_PRESSED;
        // 공유 변수에 터치 좌표 기록
        s_touchX = lx;
        s_touchY = ly;
        s_touchPressed = true;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        s_touchPressed = false;
    }
}

// ============================================================
// LVGL MUTEX HELPERS
// ============================================================

static bool lvgl_lock(int timeout_ms)
{
    const TickType_t ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mutex, ticks) == pdTRUE;
}

static void lvgl_unlock(void)
{
    xSemaphoreGive(lvgl_mutex);
}

// ============================================================
// LVGL DEDICATED TASK (matches Waveshare example)
// ============================================================

static void lvgl_port_task(void *arg)
{
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    for (;;) {
        if (lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static void init_backlight_from_example(void)
{
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_8_BIT;
    timer_conf.timer_num = LEDC_TIMER_3;
    timer_conf.freq_hz = 50 * 1000;
    timer_conf.clk_cfg = static_cast<ledc_clk_cfg_t>(LEDC_SLOW_CLK_RC_FAST);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = LCD_PIN_BL;
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = LEDC_CHANNEL_1;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = LEDC_TIMER_3;
    ch_conf.duty = LCD_PWM_MODE_255;  // Backlight ON in Waveshare polarity
    ch_conf.hpoint = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&ch_conf));

    ESP_LOGI(TAG, "Backlight initialized (Waveshare PWM mode)");
}

// Backlight runtime control
static bool s_backlightOn = true;

void setBacklight(bool on)
{
    uint32_t duty = on ? LCD_PWM_MODE_255 : LCD_PWM_MODE_0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    s_backlightOn = on;
}

bool isBacklightOn(void) { return s_backlightOn; }

// ============================================================
// LCD INITIALIZATION
// ============================================================

void initPowerLatch(void)
{
    // LCD RST LOW 즉시 — cold boot 깨진 화면 방지
    gpio_config_t rst_conf = {};
    rst_conf.intr_type = GPIO_INTR_DISABLE;
    rst_conf.mode = GPIO_MODE_OUTPUT;
    rst_conf.pin_bit_mask = (1ULL << LCD_PIN_RST);
    rst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&rst_conf));
    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_RST, 0));

    // I2C 버스 + IO expander → 전원 래치 (EXIO6/EXIO7 HIGH)
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.i2c_port = IOEXP_I2C_PORT;
    bus_cfg.sda_io_num = IOEXP_PIN_SDA;
    bus_cfg.scl_io_num = IOEXP_PIN_SCL;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_sensor_bus);
    if (err == ESP_OK) {
        err = esp_io_expander_new_i2c_tca9554(
            s_sensor_bus,
            ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
            &s_io_expander);
        if (err == ESP_OK) {
            esp_io_expander_set_dir(s_io_expander,
                                    IO_EXPANDER_PIN_NUM_7 | IO_EXPANDER_PIN_NUM_6,
                                    IO_EXPANDER_OUTPUT);
            esp_io_expander_set_level(s_io_expander,
                                      IO_EXPANDER_PIN_NUM_7 | IO_EXPANDER_PIN_NUM_6,
                                      1);
            ESP_LOGI(TAG, "Power latch: ON");
        } else {
            ESP_LOGW(TAG, "IO expander init failed: %d", (int)err);
        }
    } else {
        ESP_LOGW(TAG, "Sensor I2C bus init failed: %d", (int)err);
    }
}

static void init_lcd_hardware(void)
{
    ESP_LOGI(TAG, "Initializing LCD hardware...");
    ESP_LOGI(TAG, "LCD pins: CS=%d PCLK=%d D0=%d D1=%d D2=%d D3=%d RST=%d BL=%d",
             LCD_PIN_CS, LCD_PIN_SCLK, LCD_PIN_DATA0, LCD_PIN_DATA1, LCD_PIN_DATA2, LCD_PIN_DATA3, LCD_PIN_RST, LCD_PIN_BL);

    // initPowerLatch()에서 이미 RST LOW + I2C + IO expander 완료된 경우 스킵
    if (!s_sensor_bus) {
        gpio_config_t rst_conf = {};
        rst_conf.intr_type = GPIO_INTR_DISABLE;
        rst_conf.mode = GPIO_MODE_OUTPUT;
        rst_conf.pin_bit_mask = (1ULL << LCD_PIN_RST);
        rst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        rst_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        ESP_ERROR_CHECK(gpio_config(&rst_conf));
        ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_RST, 0));

        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.i2c_port = IOEXP_I2C_PORT;
        bus_cfg.sda_io_num = IOEXP_PIN_SDA;
        bus_cfg.scl_io_num = IOEXP_PIN_SCL;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;
        esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_sensor_bus);
        if (err == ESP_OK) {
            err = esp_io_expander_new_i2c_tca9554(
                s_sensor_bus,
                ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                &s_io_expander);
            if (err == ESP_OK) {
                esp_io_expander_set_dir(s_io_expander,
                                        IO_EXPANDER_PIN_NUM_7 | IO_EXPANDER_PIN_NUM_6,
                                        IO_EXPANDER_OUTPUT);
                esp_io_expander_set_level(s_io_expander,
                                          IO_EXPANDER_PIN_NUM_7 | IO_EXPANDER_PIN_NUM_6,
                                          1);
                ESP_LOGI(TAG, "IO expander power enabled");
            } else {
                ESP_LOGW(TAG, "IO expander init failed: %d", (int)err);
            }
        } else {
            ESP_LOGW(TAG, "IO expander I2C bus init failed: %d", (int)err);
        }
    } else {
        ESP_LOGI(TAG, "IO expander already initialized (power latch)");
    }

    // Wait for LCD power rails to stabilize after IO expander enables them.
    // On cold boot the LCD controller needs time before it can accept a clean
    // reset pulse; without this delay the first power-on often fails.
    vTaskDelay(pdMS_TO_TICKS(500));

    // Backlight control exactly follows Waveshare example behavior.
    init_backlight_from_example();

    // SPI Bus
    if (!s_flush_done_semaphore) {
        s_flush_done_semaphore = xSemaphoreCreateBinary();
    }
    if (!s_flush_done_semaphore) {
        ESP_LOGE(TAG, "Failed to create LCD flush semaphore");
        return;
    }

    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = LCD_PIN_SCLK;
    buscfg.data0_io_num = LCD_PIN_DATA0;
    buscfg.data1_io_num = LCD_PIN_DATA1;
    buscfg.data2_io_num = LCD_PIN_DATA2;
    buscfg.data3_io_num = LCD_PIN_DATA3;
    // Match Waveshare example: transfer in native-width DMA chunks.
    buscfg.max_transfer_sz = LCD_DMA_BUF_SIZE;
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // LCD Panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = LCD_PIN_CS;
    io_config.dc_gpio_num = -1;
    io_config.spi_mode = 3;
    io_config.pclk_hz = 40 * 1000 * 1000;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = lcd_trans_done_cb;
    io_config.user_ctx = s_flush_done_semaphore;
    io_config.lcd_cmd_bits = 32;
    io_config.lcd_param_bits = 8;
    io_config.cs_ena_pretrans = 0;
    io_config.cs_ena_posttrans = 0;
    io_config.flags.quad_mode = 1;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &io_handle));

    // LCD Panel (AXS15231B)
    // Matches Waveshare V9 example exactly: Sleep Out + Display On.
    static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
        {0x11, (uint8_t[]){0x00}, 0, 100},   // Sleep out
        {0x29, (uint8_t[]){0x00}, 0, 100},   // Display on
    };
    axs15231b_vendor_config_t vendor_config = {};
    vendor_config.flags.use_qspi_interface = 1;
    vendor_config.init_cmds = lcd_init_cmds;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]);

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = -1;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    panel_config.vendor_config = &vendor_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(io_handle, &panel_config, &panel_handle));

    // Release LCD from reset.  RST has been held LOW since the top of this
    // function, so the controller is in a known reset state regardless of
    // whether this is a cold boot or warm reboot.
    // AXS15231B needs ≥120ms after RST release before accepting commands.
    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_LOGI(TAG, "LCD initialized: %dx%d native", LCD_NATIVE_H, LCD_NATIVE_V);
}

// ============================================================
// TOUCH INITIALIZATION
// ============================================================

static void init_touch_hardware(void)
{
    ESP_LOGI(TAG, "Initializing touch controller...");

#if TOUCH_PIN_RST >= 0
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << TOUCH_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&rst_conf);

    gpio_set_level((gpio_num_t)TOUCH_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)TOUCH_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
#endif

    // I2C for touch (new driver)
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.i2c_port = TOUCH_I2C_PORT;
    bus_cfg.sda_io_num = TOUCH_PIN_SDA;
    bus_cfg.scl_io_num = TOUCH_PIN_SCL;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_touch_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Touch I2C bus init failed: %d", (int)err);
        return;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = CST816S_ADDR;
    dev_cfg.scl_speed_hz = 300000;
    err = i2c_master_bus_add_device(s_touch_bus, &dev_cfg, &s_touch_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Touch I2C device init failed: %d", (int)err);
        return;
    }

    ESP_LOGI(TAG, "Touch initialized");
}

// ============================================================
// LVGL INITIALIZATION
// ============================================================

static void init_lvgl(void)
{
    ESP_LOGI(TAG, "Initializing LVGL...");

    lv_init();

    // Match Waveshare example: create in native size and use software rotation for landscape.
    lvgl_disp = lv_display_create(LCD_NATIVE_H, LCD_NATIVE_V);
    lv_display_set_flush_cb(lvgl_disp, lvgl_flush_cb);

    // Follow Waveshare example: full-screen double buffer + rotate buffer in PSRAM.
    size_t buf_size = LCD_FULL_BUF_SIZE;
    void *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    void *buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    s_lvgl_rot_buf = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    s_lvgl_dma_buf = (uint16_t *)heap_caps_malloc(LCD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    if (!buf1 || !buf2 || !s_lvgl_rot_buf || !s_lvgl_dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate LVGL buffers (PSRAM/DMA)");
        return;
    }
    lv_display_set_buffers(lvgl_disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_rotation(lvgl_disp, kDisplayRotation);

    // Create tick timer
    esp_timer_create_args_t tick_timer_args = {};
    tick_timer_args.callback = lvgl_tick_timer_cb;
    tick_timer_args.arg = nullptr;
    tick_timer_args.dispatch_method = ESP_TIMER_TASK;
    tick_timer_args.skip_unhandled_events = false;
    tick_timer_args.name = "lvgl_tick";
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    // Create mutex for thread safety
    lvgl_mutex = xSemaphoreCreateMutex();

    // Touch input device (LVGL indev - Waveshare example protocol)
    lv_indev_t *touch_indev = lv_indev_create();
    lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch_indev, lvgl_touch_read_cb);

    // Dedicated LVGL task (Core 0, matches Waveshare example)
    xTaskCreatePinnedToCore(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE,
                            NULL, LVGL_TASK_PRIORITY, NULL, 0);

    ESP_LOGI(TAG, "LVGL initialized (dedicated task on Core 0)");
}

// ============================================================
// UI CREATION
// ============================================================

void setupUI(void)
{
    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) != pdTRUE) return;

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // Delta bar background - covers entire screen (640x172)
    // Green bar: grows from left to right (faster)
    // Red bar: grows from right to left (slower)
    bar_bg = lv_obj_create(scr);
    lv_obj_set_size(bar_bg, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(bar_bg, 0, 0);
    lv_obj_set_style_bg_color(bar_bg, lv_color_black(), 0);
    lv_obj_set_style_border_width(bar_bg, 0, 0);
    lv_obj_set_style_pad_all(bar_bg, 0, 0);
    lv_obj_set_style_radius(bar_bg, 0, 0);

    // ── 저전력 경고 텍스트 (하단 중앙) ──
    lbl_bat_low = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_bat_low, lv_color_hex(0x551111), 0);
    lv_obj_set_style_text_font(lbl_bat_low, &lv_font_montserrat_14, 0);
    lv_label_set_text(lbl_bat_low, "BAT LOW");
    lv_obj_align(lbl_bat_low, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(lbl_bat_low, LV_OBJ_FLAG_HIDDEN);

    // Green bar (faster) - starts from left, grows right
    bar_up = lv_obj_create(bar_bg);
    lv_obj_set_style_bg_color(bar_up, lv_color_make(20, 170, 20), 0);
    // Bright border at the moving edge
    lv_obj_set_style_border_color(bar_up, lv_color_make(40, 220, 40), 0);
    lv_obj_set_style_border_width(bar_up, 2, 0);
    lv_obj_set_style_border_side(bar_up, LV_BORDER_SIDE_RIGHT, 0);
    lv_obj_set_style_radius(bar_up, 0, 0);
    lv_obj_set_size(bar_up, 0, LCD_V_RES);
    lv_obj_set_pos(bar_up, 0, 0);

    // Red bar (slower) - starts from right, grows left
    bar_down = lv_obj_create(bar_bg);
    lv_obj_set_style_bg_color(bar_down, lv_color_make(170, 20, 20), 0);
    // Bright border at the moving edge
    lv_obj_set_style_border_color(bar_down, lv_color_make(220, 40, 40), 0);
    lv_obj_set_style_border_width(bar_down, 2, 0);
    lv_obj_set_style_border_side(bar_down, LV_BORDER_SIDE_LEFT, 0);
    lv_obj_set_style_radius(bar_down, 0, 0);
    lv_obj_set_size(bar_down, 0, LCD_V_RES);
    lv_obj_set_pos(bar_down, LCD_H_RES, 0);

    // 날짜/시간 (delta 위, 중앙, 24pt 흰색)
    lbl_datetime = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_datetime, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_datetime, &share_tech_mono_24, 0);
    lv_obj_set_width(lbl_datetime, 300);
    lv_obj_set_style_text_align(lbl_datetime, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_datetime, LV_ALIGN_TOP_MID, 0, 8);
    lv_label_set_text(lbl_datetime, "");
    lv_obj_add_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);

    // Delta text - largest, centered (overlay on bar)
    lbl_delta = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_delta, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_delta, &share_tech_mono_56, 0);
    lv_obj_set_width(lbl_delta, 420);
    lv_obj_set_style_text_align(lbl_delta, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_delta, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(lbl_delta, "0:00.0");

    // Speed delta text attached to moving bar end
    lbl_speed_delta = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_speed_delta, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_speed_delta, &share_tech_mono_24, 0);
    lv_label_set_text(lbl_speed_delta, "+0.0km/h");
    lv_obj_add_flag(lbl_speed_delta, LV_OBJ_FLAG_HIDDEN);

    // ============================================================
    // MAIN UI ELEMENTS
    // ============================================================

    // Lap tag (top-left): "LAP 01"
    lbl_lapnum = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_lapnum, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_lapnum, &share_tech_mono_32, 0);
    lv_obj_align(lbl_lapnum, LV_ALIGN_TOP_LEFT, 8, 4);
    lv_label_set_text(lbl_lapnum, "LAP 01");

    // Lap time label (top-right)
    lbl_laptime = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_laptime, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_laptime, &share_tech_mono_32, 0);
    lv_obj_set_width(lbl_laptime, 230);
    lv_obj_set_style_text_align(lbl_laptime, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(lbl_laptime, LV_ALIGN_TOP_RIGHT, -8, 4);
    lv_label_set_text(lbl_laptime, "0:00.0");
    lv_obj_add_flag(lbl_laptime, LV_OBJ_FLAG_HIDDEN);

    // Best lap label (fixed row under lap row)
    lbl_best = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_best, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_best, &share_tech_mono_24, 0);
    lv_obj_set_width(lbl_best, 320);
    lv_obj_set_style_text_align(lbl_best, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(lbl_best, LV_ALIGN_TOP_RIGHT, -8, 44);
    lv_label_set_text(lbl_best, "BEST --:--:--");
    lv_obj_add_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);

    // Sector delta labels (under LAP number on the left)
    const char* sector_names[3] = {"S1", "S2", "S3"};
    for (int i = 0; i < 3; i++) {
        lbl_sector_deltas[i] = lv_label_create(scr);
        lv_obj_set_style_text_color(lbl_sector_deltas[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl_sector_deltas[i], &share_tech_mono_24, 0);
        lv_obj_set_width(lbl_sector_deltas[i], 120);
        lv_obj_set_style_text_align(lbl_sector_deltas[i], LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(lbl_sector_deltas[i], LV_ALIGN_TOP_LEFT, 8, 42 + (i * 26));
        char buf[16];
        snprintf(buf, sizeof(buf), "%s --.--", sector_names[i]);
        lv_label_set_text(lbl_sector_deltas[i], buf);
        lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Notification label (bottom center, hidden by default)
    lbl_notification = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl_notification, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_notification, &share_tech_mono_24, 0);
    lv_obj_set_style_bg_color(lbl_notification, lv_color_make(0, 80, 0), 0);
    lv_obj_set_style_bg_opa(lbl_notification, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(lbl_notification, 8, 0);
    lv_obj_align(lbl_notification, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_flag(lbl_notification, LV_OBJ_FLAG_HIDDEN);

    // GPS Status page labels (5 lines, Share Tech Mono 24pt)
    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        gps_status_labels[i] = lv_label_create(scr);
        lv_obj_set_style_text_color(gps_status_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(gps_status_labels[i], &share_tech_mono_24, 0);
        lv_obj_set_width(gps_status_labels[i], 620);
        lv_obj_set_style_text_align(gps_status_labels[i], LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(gps_status_labels[i], LV_ALIGN_TOP_LEFT, 10, 6 + (i * 32));
        lv_label_set_text(gps_status_labels[i], "");
        lv_obj_add_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Lap completion fullscreen overlay (hidden by default)
    lap_complete_overlay = lv_obj_create(scr);
    lv_obj_set_size(lap_complete_overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(lap_complete_overlay, 0, 0);
    lv_obj_set_style_bg_color(lap_complete_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lap_complete_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(lap_complete_overlay, 0, 0);
    lv_obj_set_style_pad_all(lap_complete_overlay, 0, 0);
    lv_obj_set_style_radius(lap_complete_overlay, 0, 0);
    lv_obj_add_flag(lap_complete_overlay, LV_OBJ_FLAG_HIDDEN);

    // Lap completion time label (centered on overlay)
    lap_complete_time_label = lv_label_create(lap_complete_overlay);
    lv_obj_set_style_text_color(lap_complete_time_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(lap_complete_time_label, &share_tech_mono_56, 0);
    lv_obj_set_width(lap_complete_time_label, 420);
    lv_obj_set_style_text_align(lap_complete_time_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lap_complete_time_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(lap_complete_time_label, "");

    // Lap completion border (for best lap indication)
    lap_complete_border = lv_obj_create(lap_complete_overlay);
    lv_obj_set_size(lap_complete_border, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(lap_complete_border, 0, 0);
    lv_obj_set_style_bg_opa(lap_complete_border, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(lap_complete_border, lv_color_make(20, 170, 20), 0);
    lv_obj_set_style_border_width(lap_complete_border, 6, 0);
    lv_obj_set_style_radius(lap_complete_border, 0, 0);
    lv_obj_set_style_pad_all(lap_complete_border, 0, 0);
    lv_obj_add_flag(lap_complete_border, LV_OBJ_FLAG_HIDDEN);

    xSemaphoreGive(lvgl_mutex);

    ESP_LOGI(TAG, "UI created (landscape 640x172)");
}

// ============================================================
// INITIALIZATION
// ============================================================

void initDisplay(void)
{
    tframe.hours = 0xFF;  // 시간 미설정 상태 (GPS 시간 수신 전까지 "--:--" 표시)
    init_lcd_hardware();
    init_touch_hardware();
    init_lvgl();

    ESP_LOGI(TAG, "Display initialized: %dx%d", LCD_H_RES, LCD_V_RES);
}

// ============================================================
// UPDATE FUNCTIONS
// ============================================================

void updateLapData(void)
{
    // ─── PRE_TRACK 모드: 속도 + 현재 시각 표시 (세션 시작 전) ───
    if (s_preTrackMode) {
        // GPS 속도 (gApp.currentPoint는 gps_processor에서 GPS 수신 시마다 업데이트)
        int speedKmh = (int)gApp.currentPoint.speedKmh;
        char speedBuf[16];
        snprintf(speedBuf, sizeof(speedBuf), "%d km/h", speedKmh);

        // 하단 텍스트: 트랙 식별됨이면 트랙 이름, 아니면 현재 시각
        char bottomBuf[72] = "";
        uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
        bool showCalibMsg = (gApp.fusionCalibDoneMs > 0 &&
                             (nowMs - gApp.fusionCalibDoneMs) < 3000);

        if (showCalibMsg) {
            snprintf(bottomBuf, sizeof(bottomBuf), "IMU Calibrated");
        } else if (s_preTrackName[0] != '\0') {
            // 트랙 이름 표시
            snprintf(bottomBuf, sizeof(bottomBuf), ">> %s <<", s_preTrackName);
        } else {
            // PRE_TRACK: 현재 시각 (HH:MM:SS)
            time_t now_t = time(nullptr);
            struct tm tmInfo = {};
            localtime_r(&now_t, &tmInfo);
            snprintf(bottomBuf, sizeof(bottomBuf), "%02d:%02d:%02d",
                     tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec);
        }

        if (lvgl_lock(10)) {
            if (lbl_delta)    lv_label_set_text(lbl_delta, speedBuf);
            if (lbl_datetime) {
                lv_label_set_text(lbl_datetime, bottomBuf);
                lv_obj_clear_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
            }
            lvgl_unlock();
        }
        lvalid = false;
        return;
    }

    // ═══ Phase 1: Data Preparation (mutex 불필요) ═══
    // 모든 연산/포맷팅을 먼저 수행하여 mutex 점유 시간 최소화

    bool isFirstLap = (tframe.lap <= 1);
    bool hasReference = gApp.hasValidReferenceLap && !isFirstLap;

    // Lap completion overlay 데이터
    bool lapCompleteMode = (lframe.lapCompleteDisplayEndMs > 0 &&
                           (esp_timer_get_time() / 1000) < lframe.lapCompleteDisplayEndMs);
    char lcTimeBuf[16] = "";
    bool lcIsBest = false;
    if (lapCompleteMode) {
        uint32_t ct = lframe.lastCompletedLapMs;
        lcIsBest = (lframe.hasBestLap && ct <= lframe.bestLapMs);
        uint32_t mins = ct / 60000;
        uint32_t secs = (ct / 1000) % 60;
        uint32_t ms = (ct % 1000) / 10;
        snprintf(lcTimeBuf, sizeof(lcTimeBuf), "%lu:%02lu.%02lu", mins, secs, ms);
    }

    // EMA 스무딩
    float rawDelta = lframe.delta / 1000.0f;
    if (cache.delta[0] == '\0') {
        smoothedDelta = rawDelta;
    } else {
        smoothedDelta = DELTA_EMA_ALPHA * rawDelta + (1.0f - DELTA_EMA_ALPHA) * smoothedDelta;
    }

    // 랩타임 & 델타 텍스트 포맷팅
    uint32_t t = lframe.time;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%lu:%02lu.%lu",
             t / 60000, (t / 1000) % 60, (t % 1000) / 100);

    char deltaBuf[16];
    snprintf(deltaBuf, sizeof(deltaBuf), "%+.2f", smoothedDelta);

    // 페이지 설정에 따른 중앙/보조 텍스트 결정
    const PageConfig& pc = getDisplayConfig().pages[s_userPageIndex];
    char speedCenterBuf[16];
    CenterContent effectiveCenter = pc.center;
    if (!hasReference && effectiveCenter == CenterContent::DELTA) {
        effectiveCenter = CenterContent::LAPTIME;  // 레퍼런스 없으면 델타 대신 랩타임
    }
    const char *centerText;
    const char *laptimeText;
    switch (effectiveCenter) {
        case CenterContent::SPEED:
            snprintf(speedCenterBuf, sizeof(speedCenterBuf), "%d", (int)gApp.currentPoint.speedKmh);
            centerText  = speedCenterBuf;
            laptimeText = timeBuf;
            break;
        case CenterContent::LAPTIME:
            centerText  = timeBuf;
            laptimeText = hasReference ? deltaBuf : "Ref Lap";
            break;
        case CenterContent::DELTA:
        default:
            centerText  = deltaBuf;
            laptimeText = timeBuf;
            break;
    }

    // Lap number
    char lapBuf[16];
    snprintf(lapBuf, sizeof(lapBuf), "Lap%02u", (unsigned)tframe.lap);

    // Top 3 laps
    char top3Buf[64] = "";
    int validCount = 0;
    for (int i = 0; i < 3; i++) {
        if (gApp.top3Laps[i].lapNumber > 0 && gApp.top3Laps[i].lapTimeMs < UINT32_MAX) {
            char line[24];
            uint32_t lt = gApp.top3Laps[i].lapTimeMs;
            snprintf(line, sizeof(line), "L%02u %lu:%02lu.%02lu\n",
                     gApp.top3Laps[i].lapNumber,
                     lt / 60000, (lt / 1000) % 60, (lt % 1000) / 10);
            strncat(top3Buf, line, sizeof(top3Buf) - strlen(top3Buf) - 1);
            validCount++;
        }
    }
    if (validCount == 0) {
        snprintf(top3Buf, sizeof(top3Buf), "L-- --:--.--");
    }

    // Speed delta EMA
    if (gApp.currentDelta.hasSpeedDelta) {
        float rawSpeedDeltaKmh = gApp.currentDelta.speedDeltaKmh;
        smoothedSpeedDeltaKmh = SPEED_DELTA_EMA_ALPHA * rawSpeedDeltaKmh
                                + (1.0f - SPEED_DELTA_EMA_ALPHA) * smoothedSpeedDeltaKmh;
    } else {
        smoothedSpeedDeltaKmh *= 0.92f;
        if (fabsf(smoothedSpeedDeltaKmh) < 0.05f) {
            smoothedSpeedDeltaKmh = 0.0f;
        }
    }

    // Bar 치수 계산 (bar 모드에 따라 time 또는 speed 기반)
    const int screen_w = LCD_H_RES;
    const int screen_h = LCD_V_RES;
    int32_t barValue;
    float ratio;
    bool faster, slower;
    switch (pc.bar) {
        case BarMode::TIME: {
            float rangeMs = DELTA_BAR_RANGE_SECONDS * 1000.0f;
            float deltaMs = (float)lframe.delta;
            if (deltaMs >  rangeMs) deltaMs =  rangeMs;
            if (deltaMs < -rangeMs) deltaMs = -rangeMs;
            ratio  = fabsf(deltaMs) / rangeMs;
            faster = hasReference && (lframe.delta < -50);   // 50ms deadzone
            slower = hasReference && (lframe.delta >  50);
            barValue = lframe.delta;
            break;
        }
        case BarMode::SPEED:
        default: {
            ratio  = fabsf(smoothedSpeedDeltaKmh) / SPEED_BAR_RANGE_KMH;
            if (ratio > 1.0f) ratio = 1.0f;
            faster  = hasReference && (smoothedSpeedDeltaKmh >  SPEED_BAR_DEADZONE_KMH);
            slower  = hasReference && (smoothedSpeedDeltaKmh < -SPEED_BAR_DEADZONE_KMH);
            barValue = (int32_t)(smoothedSpeedDeltaKmh * 100.0f);
            break;
        }
    }
    int fill = (int)(ratio * (float)screen_w);
    if (fill < 0) fill = 0;
    if (fill > screen_w) fill = screen_w;
    bool showSpeedDelta = (pc.bar == BarMode::SPEED) && hasReference && (faster || slower);

    // Speed delta 텍스트
    char speedBuf[24] = "";
    if (showSpeedDelta) {
        snprintf(speedBuf, sizeof(speedBuf), "%+.1f", smoothedSpeedDeltaKmh);
    }

    // Sector 데이터 준비
    const CurrentSectorTiming& sectorTiming = getCurrentSectorTiming();
    bool showSectors = (sectorTiming.totalSectors > 0 && hasReference && pc.showSectors);
    int totalSectors = sectorTiming.totalSectors;

    struct { char text[32]; bool show; } sectorData[3] = {};
    if (showSectors) {
        const char* sector_names[3] = {"S1", "S2", "S3"};
        float totalDeltaSec = gApp.currentDelta.deltaSeconds;
        float completedCumulativeDelta = 0.0f;
        if (sectorTiming.completedCount > 0) {
            completedCumulativeDelta = sectorTiming.cumulativeDeltaAtExit[sectorTiming.completedCount - 1];
        }
        for (int i = 0; i < 3 && i < totalSectors; i++) {
            float delta = 0.0f;
            bool isCurrent = (i == sectorTiming.currentSector);
            if (sectorTiming.sectorCompleted[i]) {
                delta = sectorTiming.sectorDeltas[i];
            } else if (isCurrent) {
                delta = totalDeltaSec - completedCumulativeDelta;
            }
            sectorData[i].show = sectorTiming.sectorCompleted[i] || isCurrent;
            if (sectorData[i].show) {
                const char* sign = (delta >= 0) ? "+" : "";
                snprintf(sectorData[i].text, sizeof(sectorData[i].text),
                         "%s %s%.2f", sector_names[i], sign, delta);
            }
        }
    }

    // ═══ Phase 2: LVGL Widget Updates (mutex 필요) ═══
    if (!lvgl_lock(10)) return;

    // Defensive: ensure GPS status labels hidden while on main page
    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        if (gps_status_labels[i]) lv_obj_add_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Lap completion fullscreen overlay
    if (lapCompleteMode && lap_complete_overlay) {
        lv_label_set_text(lap_complete_time_label, lcTimeBuf);
        if (lcIsBest) lv_obj_clear_flag(lap_complete_border, LV_OBJ_FLAG_HIDDEN);
        else          lv_obj_add_flag(lap_complete_border, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lap_complete_overlay, LV_OBJ_FLAG_HIDDEN);
        lvgl_unlock();
        lvalid = false;
        return;
    } else if (lap_complete_overlay) {
        lv_obj_add_flag(lap_complete_overlay, LV_OBJ_FLAG_HIDDEN);
    }

    // 우상단 라벨 + 레퍼런스 유무에 따른 표시
    if (hasReference) {
        if (strcmp(laptimeText, cache.lapTime) != 0) {
            lv_label_set_text(lbl_laptime, laptimeText);
            strncpy(cache.lapTime, laptimeText, sizeof(cache.lapTime) - 1);
        }
        lv_obj_clear_flag(lbl_laptime, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);
    } else {
        if (strcmp("Ref Lap", cache.lapTime) != 0) {
            lv_label_set_text(lbl_laptime, "Ref Lap");
            strncpy(cache.lapTime, "Ref Lap", sizeof(cache.lapTime) - 1);
        }
        lv_obj_clear_flag(lbl_laptime, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);
    }

    // Lap number
    if (strcmp(lapBuf, cache.lapNum) != 0) {
        lv_label_set_text(lbl_lapnum, lapBuf);
        strcpy(cache.lapNum, lapBuf);
    }
    if (!pc.showLapNumber) lv_obj_add_flag(lbl_lapnum, LV_OBJ_FLAG_HIDDEN);

    // Top 3 laps
    if (strcmp(top3Buf, cache.best) != 0) {
        lv_label_set_text(lbl_best, top3Buf);
        strcpy(cache.best, top3Buf);
    }
    if (!pc.showBestLaps) lv_obj_add_flag(lbl_best, LV_OBJ_FLAG_HIDDEN);

    // Delta bars (NONE 모드이거나 레퍼런스 없으면 숨김)
    bool showBar = (pc.bar != BarMode::NONE) && hasReference;
    if (!showBar) {
        lv_obj_set_size(bar_up, 0, screen_h);
        lv_obj_set_size(bar_down, 0, screen_h);
        lv_obj_set_pos(bar_up, 0, 0);
        lv_obj_set_pos(bar_down, screen_w, 0);
    } else if (barValue != cache.lastBarValue && bar_bg && bar_up && bar_down) {
        if (faster) {
            lv_obj_set_size(bar_up, fill, screen_h);
            lv_obj_set_pos(bar_up, 0, 0);
            lv_obj_set_size(bar_down, 0, screen_h);
            lv_obj_set_pos(bar_down, screen_w, 0);
        } else if (slower) {
            lv_obj_set_size(bar_down, fill, screen_h);
            lv_obj_set_pos(bar_down, screen_w - fill, 0);
            lv_obj_set_size(bar_up, 0, screen_h);
            lv_obj_set_pos(bar_up, 0, 0);
        } else {
            lv_obj_set_size(bar_up, 0, screen_h);
            lv_obj_set_size(bar_down, 0, screen_h);
            lv_obj_set_pos(bar_up, 0, 0);
            lv_obj_set_pos(bar_down, screen_w, 0);
        }
        cache.lastBarValue = barValue;
    }

    // Speed delta text (위치 계산은 lv_obj_get_width 필요하므로 mutex 내부)
    if (lbl_speed_delta && showSpeedDelta) {
        if (strcmp(speedBuf, cache.speedDelta) != 0) {
            lv_label_set_text(lbl_speed_delta, speedBuf);
            strcpy(cache.speedDelta, speedBuf);
        }
        lv_obj_clear_flag(lbl_speed_delta, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_text_color(lbl_speed_delta, lv_color_white(), 0);
        lv_obj_update_layout(lbl_speed_delta);
        int txt_w = lv_obj_get_width(lbl_speed_delta);
        int txt_h = lv_obj_get_height(lbl_speed_delta);
        int y = screen_h - txt_h - 4;
        int x = 0;
        if (faster) {
            x = fill - txt_w - 8;
            if (x < 4) x = 4;
        } else {
            int edge_x = screen_w - fill;
            x = edge_x + 8;
            if (x + txt_w > screen_w - 4) x = screen_w - txt_w - 4;
        }
        lv_obj_set_pos(lbl_speed_delta, x, y);
    } else if (lbl_speed_delta) {
        lv_obj_add_flag(lbl_speed_delta, LV_OBJ_FLAG_HIDDEN);
        cache.speedDelta[0] = '\0';
    }

    // 중앙 라벨
    if (strcmp(centerText, cache.delta) != 0) {
        lv_obj_set_style_text_color(lbl_delta, lv_color_white(), 0);
        lv_label_set_text(lbl_delta, centerText);
        strncpy(cache.delta, centerText, sizeof(cache.delta) - 1);
    }
    lv_obj_clear_flag(lbl_delta, LV_OBJ_FLAG_HIDDEN);

    // Sector deltas
    if (showSectors) {
        for (int i = 0; i < 3 && i < totalSectors; i++) {
            if (sectorData[i].show) {
                lv_obj_set_style_text_color(lbl_sector_deltas[i], lv_color_white(), 0);
                lv_label_set_text(lbl_sector_deltas[i], sectorData[i].text);
                lv_obj_clear_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
        for (int i = totalSectors; i < 3; i++) {
            lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        for (int i = 0; i < 3; i++) {
            lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // 날짜/시간 표시 (시간 설정 완료 시)
    if (lbl_datetime && tframe.hours != 0xFF) {
        static char cache_datetime[24] = "";
        char dtBuf[24];
        snprintf(dtBuf, sizeof(dtBuf), "%04u/%02u/%02u %02u:%02u",
                 tframe.year, tframe.month, tframe.date,
                 tframe.hours, tframe.minutes);
        if (strcmp(dtBuf, cache_datetime) != 0) {
            lv_label_set_text(lbl_datetime, dtBuf);
            strncpy(cache_datetime, dtBuf, sizeof(cache_datetime) - 1);
        }
        lv_obj_clear_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
    } else if (lbl_datetime) {
        lv_obj_add_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
    }
    if (lbl_datetime && !pc.showDatetime) lv_obj_add_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);

    lvgl_unlock();
    lvalid = false;
}

void updateGpsData(void)
{
    gvalid = false;
}

void updateTimeData(void)
{
    // Time label is intentionally hidden in current UI layout.
    tvalid = false;
}

// ============================================================
// GPS SIGNAL STATUS
// ============================================================

static bool s_gpsSignalLost = false;

void setGpsSignalLost(bool lost)
{
    s_gpsSignalLost = lost;
}

bool isGpsSignalLost(void)
{
    return s_gpsSignalLost;
}

// ============================================================
// NOTIFICATIONS
// ============================================================

static uint64_t s_notificationEndMs = 0;
static bool s_notificationActive = false;

void showNotification(const char* message, uint16_t durationMs)
{
    if (xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    lv_label_set_text(lbl_notification, message);
    lv_obj_clear_flag(lbl_notification, LV_OBJ_FLAG_HIDDEN);

    s_notificationEndMs = (esp_timer_get_time() / 1000) + durationMs;
    s_notificationActive = true;

    xSemaphoreGive(lvgl_mutex);
}



static void updateNotification(void)
{
    if (s_notificationActive) {
        uint64_t now = esp_timer_get_time() / 1000;
        if (now >= s_notificationEndMs) {
            if (xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                lv_obj_add_flag(lbl_notification, LV_OBJ_FLAG_HIDDEN);
                xSemaphoreGive(lvgl_mutex);
            }
            s_notificationActive = false;
            cache.lapTime[0] = '\0';  // Force refresh
        }
    }
}

// ============================================================
// TOUCH HANDLING (LVGL 콜백의 공유 변수 사용 — I2C 충돌 방지)
// ============================================================

bool readTouch(void)
{
    return s_touchPressed;
}

// 제스처 감지 (좌표 기반, LVGL 공유 변수 활용)
// 맨하탄 거리로 스와이프 판정, |dx| vs |dy| 로 방향 결정
enum GestureResult : uint8_t {
    GESTURE_NONE,
    GESTURE_TAP,
    GESTURE_SWIPE_LEFT,
    GESTURE_SWIPE_RIGHT,
    GESTURE_SWIPE_UP,
    GESTURE_SWIPE_DOWN,
};

static bool    s_gestureTracking = false;
static int16_t s_gestureStartX   = 0;
static int16_t s_gestureStartY   = 0;
static int16_t s_gestureLastX    = 0;
static int16_t s_gestureLastY    = 0;


// LVGL 공유 변수 기반 제스처 감지 (매 프레임 호출)
static GestureResult detectGesture(void)
{
    bool pressed = s_touchPressed;
    int16_t cx = s_touchX;
    int16_t cy = s_touchY;

    if (pressed) {
        if (!s_gestureTracking) {
            // 터치 시작
            s_gestureTracking = true;
            s_gestureStartX = cx;
            s_gestureStartY = cy;
            s_gestureLastX = cx;
            s_gestureLastY = cy;
        } else {
            // 터치 유지 중 — 좌표 갱신
            s_gestureLastX = cx;
            s_gestureLastY = cy;
        }
        return GESTURE_NONE;  // 아직 손가락 누르고 있음
    }

    // 손가락 뗌 → 제스처 판정
    if (!s_gestureTracking) return GESTURE_NONE;
    s_gestureTracking = false;

    int16_t dx = s_gestureLastX - s_gestureStartX;
    int16_t dy = s_gestureLastY - s_gestureStartY;
    int16_t absDx = (dx >= 0) ? dx : -dx;
    int16_t absDy = (dy >= 0) ? dy : -dy;
    int16_t manhattan = absDx + absDy;

    if (manhattan < TOUCH_SWIPE_THRESHOLD_PX) {
        return GESTURE_TAP;
    }

    // Shared touch coordinates are the same raw axis mapping used by the
    // Waveshare example. For 270° display rotation (180° flip from 90°),
    // invert gesture directions to match what the user sees on screen.
    const bool flipped180 = (DISPLAY_ROTATION_DEG == 270);
    if (absDy >= absDx) {
        if (!flipped180) return (dy > 0) ? GESTURE_SWIPE_RIGHT : GESTURE_SWIPE_LEFT;
        return (dy > 0) ? GESTURE_SWIPE_LEFT : GESTURE_SWIPE_RIGHT;
    }
    if (!flipped180) return (dx > 0) ? GESTURE_SWIPE_DOWN : GESTURE_SWIPE_UP;
    return (dx > 0) ? GESTURE_SWIPE_UP : GESTURE_SWIPE_DOWN;
}

// ============================================================
// STARTUP SCREEN
// ============================================================

void createStartupScreen(void)
{
    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) != pdTRUE) return;

    lv_obj_t *scr = lv_scr_act();

    // 전체 화면 오버레이 (모든 UI 위에 렌더링)
    startup_overlay = lv_obj_create(scr);
    lv_obj_set_size(startup_overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(startup_overlay, 0, 0);
    lv_obj_set_style_bg_color(startup_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(startup_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(startup_overlay, 0, 0);
    lv_obj_set_style_pad_all(startup_overlay, 0, 0);
    lv_obj_set_style_radius(startup_overlay, 0, 0);

    // 저전력 경고 텍스트 (startup 화면)
    lbl_bat_low_startup = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(lbl_bat_low_startup, lv_color_hex(0x551111), 0);
    lv_obj_set_style_text_font(lbl_bat_low_startup, &lv_font_montserrat_14, 0);
    lv_label_set_text(lbl_bat_low_startup, "BAT LOW");
    lv_obj_align(lbl_bat_low_startup, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(lbl_bat_low_startup, LV_OBJ_FLAG_HIDDEN);

    // 프로젝트 타이틀: "GPS LAPTIMER" (56pt, 흰색, 중앙)
    startup_title_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_title_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(startup_title_label, &share_tech_mono_56, 0);
    lv_obj_set_width(startup_title_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_title_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_title_label, LV_ALIGN_TOP_MID, 0, 20);
    lv_label_set_text(startup_title_label, "GPS LAPTIMER");

    // 버전: "v0.4.0" (24pt, 회색, 중앙)
    startup_version_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_version_label, lv_color_make(0x80, 0x80, 0x80), 0);
    lv_obj_set_style_text_font(startup_version_label, &share_tech_mono_24, 0);
    lv_obj_set_width(startup_version_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_version_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_version_label, LV_ALIGN_TOP_MID, 0, 80);
    lv_label_set_text(startup_version_label, APP_VERSION);

    // GPS/모드 상태 (32pt, 중앙, 동적 색상)
    startup_status_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_status_label, lv_color_make(0xFF, 0xCC, 0x00), 0);
    lv_obj_set_style_text_font(startup_status_label, &share_tech_mono_32, 0);
    lv_obj_set_width(startup_status_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_status_label, LV_ALIGN_TOP_MID, 0, 120);
    lv_label_set_text(startup_status_label, "");

    // 모드 선택 힌트 (14pt, 회색, 중앙) — MODE_SELECT 상태에서만 표시
    startup_hint_label = lv_label_create(startup_overlay);
    lv_obj_set_style_text_color(startup_hint_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(startup_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(startup_hint_label, LCD_H_RES);
    lv_obj_set_style_text_align(startup_hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(startup_hint_label, LV_ALIGN_TOP_MID, 0, 155);
    lv_label_set_text(startup_hint_label, "");
    lv_obj_add_flag(startup_hint_label, LV_OBJ_FLAG_HIDDEN);

    // ── 전화번호 플레이트 (화면: PHONE_PLATE) ──
    phone_overlay = lv_obj_create(scr);
    lv_obj_set_size(phone_overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(phone_overlay, 0, 0);
    lv_obj_set_style_bg_color(phone_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(phone_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(phone_overlay, 0, 0);
    lv_obj_set_style_pad_all(phone_overlay, 0, 0);
    lv_obj_set_style_radius(phone_overlay, 0, 0);

    // 저전력 경고 텍스트 (phone 화면)
    lbl_bat_low_phone = lv_label_create(phone_overlay);
    lv_obj_set_style_text_color(lbl_bat_low_phone, lv_color_hex(0x551111), 0);
    lv_obj_set_style_text_font(lbl_bat_low_phone, &lv_font_montserrat_14, 0);
    lv_label_set_text(lbl_bat_low_phone, "BAT LOW");
    lv_obj_align(lbl_bat_low_phone, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(lbl_bat_low_phone, LV_OBJ_FLAG_HIDDEN);

    phone_number_label = lv_label_create(phone_overlay);
    lv_obj_set_style_text_color(phone_number_label, lv_color_make(0xFF, 0xCC, 0x00), 0);
    lv_obj_set_style_text_font(phone_number_label, &share_tech_mono_56, 0);
    lv_obj_set_width(phone_number_label, LCD_H_RES);
    lv_obj_set_style_text_align(phone_number_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(phone_number_label, LV_ALIGN_CENTER, 0, -10);
    lv_label_set_text(phone_number_label, "NO NUMBER SET");

    phone_hint_label = lv_label_create(phone_overlay);
    lv_obj_set_style_text_color(phone_hint_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(phone_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_width(phone_hint_label, LCD_H_RES);
    lv_obj_set_style_text_align(phone_hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(phone_hint_label, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_label_set_text(phone_hint_label, "swipe: back");

    lv_obj_add_flag(phone_overlay, LV_OBJ_FLAG_HIDDEN);  // 초기: 숨김

    // 배터리 상태에 따라 BAT LOW 표시
    if (gApp.batteryPercent >= 0.0f && gApp.batteryPercent < 20.0f) {
        if (lbl_bat_low_startup) lv_obj_clear_flag(lbl_bat_low_startup, LV_OBJ_FLAG_HIDDEN);
        if (lbl_bat_low_phone) lv_obj_clear_flag(lbl_bat_low_phone, LV_OBJ_FLAG_HIDDEN);
    }

    xSemaphoreGive(lvgl_mutex);

    ESP_LOGI(TAG, "Startup screen created");
}

// (updateStartupScreen, dismissStartupScreen, isStartupScreenActive,
//  isGpsStatusOnlyMode removed — replaced by Page system)

i2c_master_bus_handle_t getSensorI2CBus(void)
{
    return s_sensor_bus;
}

void systemPowerOff(void)
{
    ESP_LOGI(TAG, "System power off requested");

    // 화면에 종료 메시지 표시
    if (xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lv_obj_t *scr = lv_scr_act();

        lv_obj_t *overlay = lv_obj_create(scr);
        lv_obj_set_size(overlay, LCD_H_RES, LCD_V_RES);
        lv_obj_set_pos(overlay, 0, 0);
        lv_obj_set_style_bg_color(overlay, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(overlay, 0, 0);
        lv_obj_set_style_pad_all(overlay, 0, 0);
        lv_obj_set_style_radius(overlay, 0, 0);

        lv_obj_t *lbl = lv_label_create(overlay);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl, &share_tech_mono_32, 0);
        lv_obj_set_width(lbl, LCD_H_RES);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
        lv_label_set_text(lbl, "Shutting down...");

        lv_timer_handler();
        xSemaphoreGive(lvgl_mutex);
    }

    // 잠시 대기하여 메시지가 화면에 표시되도록 함
    vTaskDelay(pdMS_TO_TICKS(500));

    // TCA9554 PIN 6을 LOW로 → 배터리 전원 차단
    if (s_io_expander) {
        esp_io_expander_set_level(s_io_expander, IO_EXPANDER_PIN_NUM_6, 0);
        ESP_LOGI(TAG, "Power latch released - system should power off");
    } else {
        ESP_LOGW(TAG, "IO expander not available - cannot power off");
    }

    // USB 전원이면 여기까지 도달 (전원이 안 꺼짐)
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGW(TAG, "Still running - USB power connected?");
}

// ============================================================
// GPS STATUS PAGE
// ============================================================

static char s_gpsStatusCache[GPS_STATUS_LINE_COUNT][GPS_STATUS_LINE_WIDTH] = {};

/**
 * @brief GPS Status 페이지 데이터 갱신
 */
static void updateGpsStatusPage(void) {
    UBloxData ubx = getUBloxData();
    UBloxStats stats = getUBloxStats();

    // Fix type 문자열
    const char* fixStr = ubx.fixType == 0 ? "No fix" :
                         ubx.fixType == 1 ? "DR" :
                         ubx.fixType == 2 ? "2D" :
                         ubx.fixType == 3 ? "3D" :
                         ubx.fixType == 4 ? "3D+DR" : "?";

    // UART 건강도
    uint32_t totalMsgs = stats.checksumOk + stats.checksumFail;
    int healthPct = totalMsgs > 0 ? (int)(stats.checksumOk * 100 / totalMsgs) : 0;

    // 부팅 후 경과 시간 (uptime)
    int64_t uptimeUs = esp_timer_get_time();
    int uptimeSec = (int)(uptimeUs / 1000000);
    int upH = uptimeSec / 3600;
    int upM = (uptimeSec % 3600) / 60;
    int upS = uptimeSec % 60;

    // 배터리 표시
    int batPct = (gApp.batteryPercent >= 0.0f) ? (int)(gApp.batteryPercent + 0.5f) : 0;

    char lines[GPS_STATUS_LINE_COUNT][GPS_STATUS_LINE_WIDTH] = {};

    // Line 1: header + battery + uptime
    snprintf(lines[0], sizeof(lines[0]),
             "GPS STATUS  BAT:%d%% %.2fV  UPTIME:%02d:%02d:%02d",
             batPct, gApp.batteryVoltage, upH, upM, upS);

    // Line 2: fix + satellite summary (used/total)
    NavSatData sat = getNavSatData();
    int satUsedTotal = sat.usedGps + sat.usedGlo + sat.usedGal + sat.usedBds;
    int satVisibleTotal = sat.numSvs;
    if (sat.valid) {
        snprintf(lines[1], sizeof(lines[1]),
                 "FIX:%s  SAT USED/TOTAL:%d/%d  HDOP:%.1f",
                 fixStr, satUsedTotal, satVisibleTotal, ubx.hdop);
    } else {
        snprintf(lines[1], sizeof(lines[1]),
                 "FIX:%s  SAT USED/TOTAL:%d/%d  HDOP:%.1f",
                 fixStr, ubx.satellites, ubx.satellites, ubx.hdop);
    }

    // Line 3: constellation details (used/total)
    if (sat.valid) {
        snprintf(lines[2], sizeof(lines[2]),
                 "GPS:%d/%d  GLO:%d/%d  GAL:%d/%d  BDS:%d/%d",
                 sat.usedGps, sat.visGps, sat.usedGlo, sat.visGlo,
                 sat.usedGal, sat.visGal, sat.usedBds, sat.visBds);
    } else {
        snprintf(lines[2], sizeof(lines[2]),
                 "GPS:--/--  GLO:--/--  GAL:--/--  BDS:--/--");
    }

    // Line 4: coordinates + speed + altitude
    if (ubx.fixType >= 2) {
        char latDir = ubx.lat >= 0 ? 'N' : 'S';
        char lonDir = ubx.lon >= 0 ? 'E' : 'W';
        snprintf(lines[3], sizeof(lines[3]),
                 "LAT:%c%.6f LON:%c%.6f SPD:%.1f ALT:%.1f",
                 latDir, fabs(ubx.lat), lonDir, fabs(ubx.lon), ubx.speedKmh, ubx.altitudeM);
    } else {
        snprintf(lines[3], sizeof(lines[3]),
                 "LAT:--.------ LON:--.------ SPD:--.- ALT:--.-");
    }

    // Line 5: rate + UART health + UTC
    if (ubx.timeValid) {
        snprintf(lines[4], sizeof(lines[4]),
                 "RATE:%.1fHz  UART:%d%%  UTC:%02u:%02u:%02u",
                 stats.measuredHz, healthPct, ubx.hour, ubx.minute, ubx.second);
    } else {
        snprintf(lines[4], sizeof(lines[4]),
                 "RATE:%.1fHz  UART:%d%%  UTC:--:--:--",
                 stats.measuredHz, healthPct);
    }

    // 캐시 비교 후 변경 시에만 LVGL 업데이트
    if (xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    // Defensive: ensure main UI stays hidden while on GPS Status page
    lv_obj_t *hideElements[] = {bar_bg, lbl_delta, lbl_datetime, lbl_lapnum, lbl_laptime, lbl_best, lbl_speed_delta, lap_complete_overlay};
    for (auto el : hideElements) {
        if (el) lv_obj_add_flag(el, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < 3; i++) {
        if (lbl_sector_deltas[i]) lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Per-line colors for readability
    //  0: header/battery/uptime — yellow
    //  1: fix/sat summary       — green(3D), yellow(2D), red(no fix)
    //  2: constellation detail   — cyan
    //  3: coordinates/speed      — white
    //  4: rate/UART/UTC          — grey
    lv_color_t lineColors[GPS_STATUS_LINE_COUNT];
    lineColors[0] = lv_color_make(0xFF, 0xCC, 0x00);  // yellow
    if (ubx.fixType >= 3)
        lineColors[1] = lv_color_make(0x20, 0xDD, 0x20);  // green
    else if (ubx.fixType == 2)
        lineColors[1] = lv_color_make(0xFF, 0xCC, 0x00);  // yellow
    else
        lineColors[1] = lv_color_make(0xFF, 0x44, 0x44);  // red
    lineColors[2] = lv_color_make(0x00, 0xCC, 0xDD);  // cyan
    lineColors[3] = lv_color_white();
    lineColors[4] = lv_color_make(0x99, 0x99, 0x99);  // grey

    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        if (strcmp(lines[i], s_gpsStatusCache[i]) != 0) {
            lv_label_set_text(gps_status_labels[i], lines[i]);
            strncpy(s_gpsStatusCache[i], lines[i], sizeof(s_gpsStatusCache[i]) - 1);
        }
        lv_obj_set_style_text_color(gps_status_labels[i], lineColors[i], 0);
    }

    xSemaphoreGive(lvgl_mutex);
}

// ============================================================
// DELTA HISTORY
// ============================================================

void resetDeltaHistory(void)
{
    smoothedDelta = 0.0f;
    smoothedSpeedDeltaKmh = 0.0f;
    // Clear cache first, then set lastBarValue to force redraw
    memset(&cache, 0, sizeof(cache));
    cache.lastBarValue = -9999;
}

// ============================================================
// DISPLAY TEST
// ============================================================

void displayTest(void)
{
    ESP_LOGI(TAG, "=== DISPLAY TEST ===");
    ESP_LOGI(TAG, "Display size: %dx%d (landscape)", LCD_H_RES, LCD_V_RES);

    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) != pdTRUE) return;

    lv_obj_t *scr = lv_scr_act();

    // Clear screen
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // RGB test bars (horizontal, spanning full width)
    const int bar_w = LCD_H_RES / 3;
    const int bar_h = LCD_V_RES / 2;

    lv_obj_t *red_bar = lv_obj_create(scr);
    lv_obj_set_size(red_bar, bar_w, bar_h);
    lv_obj_set_pos(red_bar, 0, 0);
    lv_obj_set_style_bg_color(red_bar, lv_color_make(170, 20, 20), 0);
    lv_obj_set_style_border_width(red_bar, 0, 0);

    lv_obj_t *green_bar = lv_obj_create(scr);
    lv_obj_set_size(green_bar, bar_w, bar_h);
    lv_obj_set_pos(green_bar, bar_w, 0);
    lv_obj_set_style_bg_color(green_bar, lv_color_make(20, 170, 20), 0);
    lv_obj_set_style_border_width(green_bar, 0, 0);

    lv_obj_t *blue_bar = lv_obj_create(scr);
    lv_obj_set_size(blue_bar, bar_w, bar_h);
    lv_obj_set_pos(blue_bar, bar_w * 2, 0);
    lv_obj_set_style_bg_color(blue_bar, lv_color_make(0, 0, 255), 0);
    lv_obj_set_style_border_width(blue_bar, 0, 0);

    // Test label
    lv_obj_t *lbl = lv_label_create(scr);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl, &share_tech_mono_24, 0);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_label_set_text(lbl, "LANDSCAPE 640x172 OK");

    xSemaphoreGive(lvgl_mutex);

    ESP_LOGI(TAG, "Test pattern drawn. Check display!");
}

// ============================================================
// DISPLAY WIDGETS API (for Page system)
// ============================================================

bool lvglLock(int timeoutMs) { return lvgl_lock(timeoutMs); }
void lvglUnlock(void)        { lvgl_unlock(); }

Gesture pollGesture(void)
{
    GestureResult gr = detectGesture();
    return static_cast<Gesture>(gr);  // enum values match by design
}

void updateNotificationDisplay(void) { updateNotification(); }

void updateBatteryWarning(void)
{
    int8_t lowBat = (gApp.batteryPercent >= 0.0f && gApp.batteryPercent < 20.0f) ? 1 : 0;
    if (lowBat != cache.lastLowBat) {
        if (lvgl_lock(10)) {
            lv_obj_t *labels[] = {lbl_bat_low, lbl_bat_low_startup, lbl_bat_low_phone};
            for (int i = 0; i < 3; i++) {
                if (!labels[i]) continue;
                if (lowBat) lv_obj_clear_flag(labels[i], LV_OBJ_FLAG_HIDDEN);
                else        lv_obj_add_flag(labels[i], LV_OBJ_FLAG_HIDDEN);
            }
            lvgl_unlock();
            cache.lastLowBat = lowBat;
        }
    }
}

// ── Widget getters ──
lv_obj_t* getStartupOverlay(void)      { return startup_overlay; }
lv_obj_t* getStartupTitleLabel(void)    { return startup_title_label; }
lv_obj_t* getStartupVersionLabel(void)  { return startup_version_label; }
lv_obj_t* getStartupStatusLabel(void)   { return startup_status_label; }
lv_obj_t* getStartupHintLabel(void)     { return startup_hint_label; }

lv_obj_t* getPhoneOverlay(void)         { return phone_overlay; }
lv_obj_t* getPhoneNumberLabel(void)     { return phone_number_label; }
lv_obj_t* getPhoneHintLabel(void)       { return phone_hint_label; }

lv_obj_t* getDeltaLabel(void)           { return lbl_delta; }
lv_obj_t* getSpeedDeltaLabel(void)      { return lbl_speed_delta; }
lv_obj_t* getDatetimeLabel(void)        { return lbl_datetime; }
lv_obj_t* getLapnumLabel(void)          { return lbl_lapnum; }
lv_obj_t* getLaptimeLabel(void)         { return lbl_laptime; }
lv_obj_t* getBestLabel(void)            { return lbl_best; }
lv_obj_t* getBarBg(void)               { return bar_bg; }
lv_obj_t* getBarUp(void)               { return bar_up; }
lv_obj_t* getBarDown(void)             { return bar_down; }
lv_obj_t* getNotificationLabel(void)    { return lbl_notification; }

lv_obj_t* getSectorDeltaLabel(int index) {
    if (index < 0 || index >= 3) return NULL;
    return lbl_sector_deltas[index];
}

lv_obj_t* getGpsStatusLabel(int line) {
    if (line < 0 || line >= GPS_STATUS_LINE_COUNT) return NULL;
    return gps_status_labels[line];
}

lv_obj_t* getLapCompleteOverlay(void)    { return lap_complete_overlay; }
lv_obj_t* getLapCompleteTimeLabel(void)  { return lap_complete_time_label; }
lv_obj_t* getLapCompleteBorder(void)     { return lap_complete_border; }

lv_obj_t* getBatLowLabel(void)           { return lbl_bat_low; }
lv_obj_t* getBatLowStartupLabel(void)    { return lbl_bat_low_startup; }
lv_obj_t* getBatLowPhoneLabel(void)      { return lbl_bat_low_phone; }

// ============================================================
// DISPLAY HELPER FUNCTIONS (for Page system)
// ============================================================

void applyPageVisibilityForPage(PageId pageId)
{
    if (xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    // Hide startup overlay when entering any main display page
    if (startup_overlay) lv_obj_add_flag(startup_overlay, LV_OBJ_FLAG_HIDDEN);
    if (phone_overlay) lv_obj_add_flag(phone_overlay, LV_OBJ_FLAG_HIDDEN);

    bool isGpsPage  = (pageId == PageId::GPS_STATUS);
    bool isPreTrack = (pageId == PageId::PRE_TRACK);

    // Main UI elements: hidden on GPS_STATUS and PRE_TRACK (except lbl_delta for speed)
    lv_obj_t *mainElements[] = {
        bar_bg, lbl_delta, lbl_lapnum, lbl_laptime, lbl_best, lbl_speed_delta
    };
    for (auto el : mainElements) {
        if (!el) continue;
        if (isGpsPage) {
            lv_obj_add_flag(el, LV_OBJ_FLAG_HIDDEN);
        } else if (isPreTrack) {
            if (el == lbl_delta) lv_obj_clear_flag(el, LV_OBJ_FLAG_HIDDEN);
            else                 lv_obj_add_flag(el, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(el, LV_OBJ_FLAG_HIDDEN);
        }
    }
    for (int i = 0; i < 3; i++) {
        if (!lbl_sector_deltas[i]) continue;
        if (isGpsPage || isPreTrack) {
            lv_obj_add_flag(lbl_sector_deltas[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // GPS Status labels: only visible on GPS_STATUS page
    for (int i = 0; i < GPS_STATUS_LINE_COUNT; i++) {
        if (!gps_status_labels[i]) continue;
        if (isGpsPage) lv_obj_clear_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
        else           lv_obj_add_flag(gps_status_labels[i], LV_OBJ_FLAG_HIDDEN);
    }

    // PRE_TRACK: show datetime label for clock display
    if (lbl_datetime) {
        if (isPreTrack) lv_obj_clear_flag(lbl_datetime, LV_OBJ_FLAG_HIDDEN);
    }

    xSemaphoreGive(lvgl_mutex);

    // Reset cache to force immediate redraw on page entry
    memset(s_gpsStatusCache, 0, sizeof(s_gpsStatusCache));
    memset(&cache, 0, sizeof(cache));
    cache.lastBarValue = -9999;
    cache.lastLowBat = -1;
}

void updateGpsStatusDisplay(void)
{
    updateGpsStatusPage();
}

void updatePreTrackDisplay(const char* trackName)
{
    // Temporarily set pretrack state and call updateLapData which handles this path
    s_preTrackMode = true;
    if (trackName) {
        strncpy(s_preTrackName, trackName, sizeof(s_preTrackName) - 1);
        s_preTrackName[sizeof(s_preTrackName) - 1] = '\0';
    } else {
        s_preTrackName[0] = '\0';
    }
    updateLapData();
}

void updateLaptimerDisplay(int userPageIndex)
{
    s_preTrackMode = false;
    s_userPageIndex = userPageIndex;
    updateLapData();
}

void updateLaptimerTimeDisplay(void)
{
    updateTimeData();
}

void updateLaptimerGpsDisplay(void)
{
    updateGpsData();
}

