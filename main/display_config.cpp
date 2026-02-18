/**
 * @file display_config.cpp
 * @brief Configurable display pages — loaded from /sdcard/config/display.json
 *
 * Example config file:
 * {
 *   "pages": [
 *     {
 *       "center": "delta",
 *       "bar": "time",
 *       "show_sectors": true,
 *       "show_best_laps": true,
 *       "show_lap_number": true,
 *       "show_datetime": true
 *     },
 *     {
 *       "center": "laptime",
 *       "bar": "speed",
 *       "show_sectors": false,
 *       "show_best_laps": true,
 *       "show_lap_number": true,
 *       "show_datetime": false
 *     }
 *   ]
 * }
 */

#include "display_config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "cJSON.h"
#include "esp_log.h"

static const char* TAG = "DisplayCfg";
static const char* CONFIG_PATH = "/sdcard/config/display.json";

static DisplayConfig s_config;

// ----------------------------------------------------------------
// Defaults: 2 pages (mirrors pre-config PAGE_DELTA / PAGE_LAPTIME)
// ----------------------------------------------------------------

static void setDefaults()
{
    s_config.pageCount = 2;

    // Page 0: delta in centre, time bar, everything visible
    s_config.pages[0].center        = CenterContent::DELTA;
    s_config.pages[0].bar           = BarMode::TIME;
    s_config.pages[0].showSectors   = true;
    s_config.pages[0].showBestLaps  = true;
    s_config.pages[0].showLapNumber = true;
    s_config.pages[0].showDatetime  = true;

    // Page 1: lap time in centre, speed bar, sectors hidden
    s_config.pages[1].center        = CenterContent::LAPTIME;
    s_config.pages[1].bar           = BarMode::SPEED;
    s_config.pages[1].showSectors   = false;
    s_config.pages[1].showBestLaps  = true;
    s_config.pages[1].showLapNumber = true;
    s_config.pages[1].showDatetime  = false;

    // Fill remaining slots with sane values
    for (int i = 2; i < MAX_USER_PAGES; i++) {
        s_config.pages[i] = PageConfig{};
    }
}

// ----------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------

static CenterContent parseCenterContent(const char* s)
{
    if (!s) return CenterContent::DELTA;
    if (strcmp(s, "laptime") == 0) return CenterContent::LAPTIME;
    if (strcmp(s, "speed")   == 0) return CenterContent::SPEED;
    return CenterContent::DELTA;
}

static BarMode parseBarMode(const char* s)
{
    if (!s) return BarMode::TIME;
    if (strcmp(s, "speed") == 0) return BarMode::SPEED;
    if (strcmp(s, "none")  == 0) return BarMode::NONE;
    return BarMode::TIME;
}

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

void loadDisplayConfig()
{
    setDefaults();

    FILE* f = fopen(CONFIG_PATH, "r");
    if (!f) {
        ESP_LOGI(TAG, "No config file at %s — using 2-page defaults", CONFIG_PATH);
        return;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0 || size > 4096) {
        fclose(f);
        ESP_LOGW(TAG, "Config file size %ld invalid — using defaults", size);
        return;
    }

    char* buf = (char*)malloc((size_t)size + 1);
    if (!buf) {
        fclose(f);
        ESP_LOGE(TAG, "malloc failed — using defaults");
        return;
    }

    size_t rd = fread(buf, 1, (size_t)size, f);
    buf[rd] = '\0';
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);

    if (!root) {
        ESP_LOGW(TAG, "JSON parse error in display config — using defaults");
        return;
    }

    cJSON* pages = cJSON_GetObjectItemCaseSensitive(root, "pages");
    if (!cJSON_IsArray(pages)) {
        cJSON_Delete(root);
        ESP_LOGW(TAG, "Config: 'pages' is not an array — using defaults");
        return;
    }

    int count = cJSON_GetArraySize(pages);
    if (count < 1)           count = 1;
    if (count > MAX_USER_PAGES) count = MAX_USER_PAGES;

    s_config.pageCount = count;

    for (int i = 0; i < count; i++) {
        cJSON* page = cJSON_GetArrayItem(pages, i);
        if (!cJSON_IsObject(page)) continue;

        PageConfig& pc = s_config.pages[i];

        cJSON* center = cJSON_GetObjectItemCaseSensitive(page, "center");
        if (cJSON_IsString(center)) pc.center = parseCenterContent(center->valuestring);

        cJSON* bar = cJSON_GetObjectItemCaseSensitive(page, "bar");
        if (cJSON_IsString(bar)) pc.bar = parseBarMode(bar->valuestring);

        cJSON* sectors = cJSON_GetObjectItemCaseSensitive(page, "show_sectors");
        if (cJSON_IsBool(sectors)) pc.showSectors = cJSON_IsTrue(sectors);

        cJSON* bestLaps = cJSON_GetObjectItemCaseSensitive(page, "show_best_laps");
        if (cJSON_IsBool(bestLaps)) pc.showBestLaps = cJSON_IsTrue(bestLaps);

        cJSON* lapNum = cJSON_GetObjectItemCaseSensitive(page, "show_lap_number");
        if (cJSON_IsBool(lapNum)) pc.showLapNumber = cJSON_IsTrue(lapNum);

        cJSON* datetime = cJSON_GetObjectItemCaseSensitive(page, "show_datetime");
        if (cJSON_IsBool(datetime)) pc.showDatetime = cJSON_IsTrue(datetime);

        ESP_LOGI(TAG, "Page %d: center=%d bar=%d sec=%d best=%d lapnum=%d dt=%d",
                 i, (int)pc.center, (int)pc.bar,
                 pc.showSectors, pc.showBestLaps, pc.showLapNumber, pc.showDatetime);
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Loaded %d page(s) from %s", s_config.pageCount, CONFIG_PATH);
}

const DisplayConfig& getDisplayConfig()
{
    return s_config;
}
