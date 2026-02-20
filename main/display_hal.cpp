/**
 * @file display_hal.cpp
 * @brief Hardware Abstraction Layer for Waveshare ESP32-S3-Touch-LCD-3.49
 *
 * LCD panel, touch, backlight, LVGL infrastructure, I2C bus, power control.
 */

#include "display_hal.h"
#include "config.h"

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

// Font needed for systemPowerOff() shutdown overlay
LV_FONT_DECLARE(share_tech_mono_32);

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

// 공유 터치 상태 (LVGL 콜백에서 기록, 다른 모듈에서 읽기)
static volatile bool s_touchPressed = false;
static volatile int16_t s_touchX = 0;
static volatile int16_t s_touchY = 0;

// Backlight runtime control
static bool s_backlightOn = true;

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
    const TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
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
// INITIALIZATION
// ============================================================

void initDisplay(void)
{
    init_lcd_hardware();
    init_touch_hardware();
    init_lvgl();

    ESP_LOGI(TAG, "Display initialized: %dx%d", LCD_H_RES, LCD_V_RES);
}

// ============================================================
// TOUCH HANDLING
// ============================================================

bool readTouch(void)
{
    return s_touchPressed;
}

bool readTouchXY(int16_t* x, int16_t* y)
{
    if (!x || !y) return false;
    if (!s_touchPressed) {
        *x = 0;
        *y = 0;
        return false;
    }
    *x = s_touchX;
    *y = s_touchY;
    return true;
}

// ============================================================
// I2C SENSOR BUS
// ============================================================

i2c_master_bus_handle_t getSensorI2CBus(void)
{
    return s_sensor_bus;
}

// ============================================================
// POWER MANAGEMENT
// ============================================================

void systemPowerOff(void)
{
    ESP_LOGI(TAG, "System power off requested");

    // 화면에 종료 메시지 표시 (LVGL 미초기화 또는 lock 실패 시 건너뜀)
    if (lvgl_mutex && lvgl_lock(100)) {
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
        lvgl_unlock();
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
// PUBLIC LVGL MUTEX API
// ============================================================

bool lvglLock(int timeoutMs) { return lvgl_lock(timeoutMs); }
void lvglUnlock(void)        { lvgl_unlock(); }
