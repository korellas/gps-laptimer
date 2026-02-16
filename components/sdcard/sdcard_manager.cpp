/**
 * @file sdcard_manager.cpp
 * @brief SD card manager (SDMMC 1-bit, FAT32)
 *
 * Pins: GPIO39(CMD), GPIO40(D0), GPIO41(CLK)
 * Mount point: /sdcard
 * No auto-format — use sdcardFormat() or serial 'm' command for manual format.
 */

#include "sdcard_manager.h"

#include <cstring>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"

static const char* TAG = "SDCARD";

// ============================================================
// Pin 정의 (hardware.md Section 8)
// ============================================================

static constexpr gpio_num_t SDMMC_PIN_CMD = GPIO_NUM_39;
static constexpr gpio_num_t SDMMC_PIN_CLK = GPIO_NUM_41;
static constexpr gpio_num_t SDMMC_PIN_D0  = GPIO_NUM_40;

static constexpr char MOUNT_POINT[] = "/sdcard";

// ============================================================
// Module State
// ============================================================

static sdmmc_card_t* s_card = nullptr;
static bool s_mounted = false;

// ============================================================
// Internal Helpers
// ============================================================

static bool ensureDir(const char* path)
{
    struct stat st = {};
    if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
        return true;
    }
    return mkdir(path, 0755) == 0;
}

// ============================================================
// Public API
// ============================================================

bool sdcardInit(void)
{
    if (s_mounted) {
        return true;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;  // 자동 포맷 안 함 — 부팅 지연 방지
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SDMMC_PIN_CLK;
    slot_config.cmd = SDMMC_PIN_CMD;
    slot_config.d0  = SDMMC_PIN_D0;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config,
                                             &mount_config, &s_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGW(TAG, "Mount failed (filesystem error — use 'm' command to format)");
        } else {
            ESP_LOGW(TAG, "Mount failed: %s", esp_err_to_name(ret));
        }
        s_card = nullptr;
        s_mounted = false;
        return false;
    }

    s_mounted = true;

    // 카드 정보 로그
    ESP_LOGI(TAG, "SD card mounted at %s", MOUNT_POINT);
    sdmmc_card_print_info(stdout, s_card);

    // laps 디렉토리 생성
    ensureDir("/sdcard/laps");

    return true;
}

void sdcardDeinit(void)
{
    if (!s_mounted) return;

    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
    s_card = nullptr;
    s_mounted = false;
    ESP_LOGI(TAG, "SD card unmounted");
}

bool sdcardFormat(void)
{
    ESP_LOGI(TAG, "Formatting SD card...");

    // 언마운트 후 재마운트 (format_if_mount_failed로 포맷 유도)
    if (s_mounted) {
        sdcardDeinit();
    }

    // 마운트 시도 — 실패하면 포맷
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = true;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SDMMC_PIN_CLK;
    slot_config.cmd = SDMMC_PIN_CMD;
    slot_config.d0  = SDMMC_PIN_D0;

    // 직접 포맷을 위해 먼저 raw mount
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config,
                                             &mount_config, &s_card);
    if (ret == ESP_OK && s_card) {
        // 이미 마운트됨 → 포맷 실행
        ret = esp_vfs_fat_sdcard_format(MOUNT_POINT, s_card);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Format failed: %s", esp_err_to_name(ret));
            esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
            s_card = nullptr;
            s_mounted = false;
            return false;
        }
        s_mounted = true;
        ensureDir("/sdcard/laps");
        ESP_LOGI(TAG, "Format complete");
        return true;
    }

    ESP_LOGE(TAG, "Cannot format: mount failed (%s)", esp_err_to_name(ret));
    s_card = nullptr;
    s_mounted = false;
    return false;
}

bool sdcardIsMounted(void)
{
    return s_mounted;
}

uint64_t sdcardGetFreeBytes(void)
{
    if (!s_mounted) return 0;

    FATFS* fs;
    DWORD fre_clust;
    if (f_getfree("0:", &fre_clust, &fs) != FR_OK) {
        return 0;
    }
    uint64_t freeBytes = (uint64_t)fre_clust * fs->csize * 512;
    return freeBytes;
}

uint64_t sdcardGetTotalBytes(void)
{
    if (!s_mounted) return 0;

    FATFS* fs;
    DWORD fre_clust;
    if (f_getfree("0:", &fre_clust, &fs) != FR_OK) {
        return 0;
    }
    uint64_t totalBytes = (uint64_t)(fs->n_fatent - 2) * fs->csize * 512;
    return totalBytes;
}
