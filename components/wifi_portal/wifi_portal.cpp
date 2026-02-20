/**
 * @file wifi_portal.cpp
 * @brief WiFi Portal — SoftAP + HTTP + WebSocket log
 *
 * - SoftAP SSID: "LAPTIMER" (open, no password)
 * - HTTP GET /              -> 로그 뷰어
 * - HTTP GET /settings      -> 설정 페이지
 * - HTTP GET /ota           -> OTA 업데이트 페이지
 * - HTTP GET /api/settings  -> JSON 설정 읽기
 * - HTTP POST /api/settings -> JSON 설정 저장
 * - HTTP POST /api/ota/upload -> 펌웨어 바이너리 업로드
 * - HTTP GET /api/ota/status  -> OTA 상태 JSON
 * - WS /ws/log              -> WebSocket 로그 스트리밍
 *
 * WiFi는 기본 OFF. startWifiPortal()로 시작, 5분 무접속 시 자동 OFF.
 * 접속: 192.168.4.1 직접 입력
 */

#include <cstdio>
#include <cstring>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

#include <dirent.h>

#include "types.h"
#include "wifi_portal.h"
#include "log_ringbuffer.h"
#include "wifi_portal_html.h"
#include "ota_manager.h"
#include "sdcard_manager.h"

static const char *TAG = "wifi_portal";

// ============================================================
// 전역 상태
// ============================================================

static httpd_handle_t s_httpServer = nullptr;
static LogRingBuffer s_logRing;
static vprintf_like_t s_origVprintf = nullptr;
static TimerHandle_t s_logFlushTimer = nullptr;

static constexpr const char *SETTINGS_PATH = "/spiffs/config/settings.json";
static constexpr int WS_MAX_CLIENTS = 4;
static constexpr bool ENABLE_LIVE_LOG_WS = false;  // 내부 RAM 부족으로 비활성화

// WiFi ON/OFF 상태
static bool s_wifiActive = false;
static bool s_wifiInitialized = false;  // 1회 초기화 완료 여부
static esp_netif_t *s_apNetif = nullptr;

// 클라이언트 활동 추적 (접속 여부 판단용)
static uint64_t s_lastActivityUs = 0;

static void updateActivity(void) {
    s_lastActivityUs = esp_timer_get_time();
}

// ============================================================
// 로그 후킹 (esp_log_set_vprintf)
// ============================================================

static int logHookVprintf(const char *fmt, va_list args)
{
    // 재진입 방지 (중복 출력 차단)
    static bool s_inHook = false;
    if (s_inHook) {
        return s_origVprintf ? s_origVprintf(fmt, args) : 0;
    }
    s_inHook = true;

    // 원본 출력 (시리얼 콘솔)
    int ret = 0;
    if (s_origVprintf) {
        va_list copy;
        va_copy(copy, args);
        ret = s_origVprintf(fmt, copy);
        va_end(copy);
    }

    // 링 버퍼에 기록 (논블로킹 — 실패 시 드롭)
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
        s_logRing.tryWrite(buf, len);
    }

    s_inHook = false;
    return ret;
}

// ============================================================
// WebSocket 브로드캐스트
// ============================================================

struct WsBroadcastArg {
    char data[512];
    size_t len;
};

// HTTPD 스레드에서 실행 (httpd_queue_work 콜백)
static void wsBroadcastOnHttpdThread(void *arg)
{
    WsBroadcastArg *ba = (WsBroadcastArg *)arg;
    if (!s_httpServer || ba->len == 0) {
        free(ba);
        return;
    }

    size_t fds_num = WS_MAX_CLIENTS;
    int fds[WS_MAX_CLIENTS];
    if (httpd_get_client_list(s_httpServer, &fds_num, fds) != ESP_OK) {
        free(ba);
        return;
    }

    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t *)ba->data;
    frame.len = ba->len;

    for (size_t i = 0; i < fds_num; i++) {
        if (httpd_ws_get_fd_info(s_httpServer, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(s_httpServer, fds[i], &frame);
        }
    }
    free(ba);
}

// 타이머 콜백: 200ms 주기로 ring buffer -> WS broadcast + 유휴 체크
static void logFlushTimerCallback(TimerHandle_t timer)
{
    if (!s_httpServer) return;

    // 로그 플러시
    if (s_logRing.hasData()) {
        WsBroadcastArg *ba = (WsBroadcastArg *)calloc(1, sizeof(WsBroadcastArg));
        if (ba) {
            ba->len = s_logRing.read(ba->data, sizeof(ba->data) - 1);
            if (ba->len == 0) {
                free(ba);
            } else {
                ba->data[ba->len] = '\0';
                if (httpd_queue_work(s_httpServer, wsBroadcastOnHttpdThread, ba) != ESP_OK) {
                    free(ba);
                }
            }
        }
    }

}

// ============================================================
// HTTP 핸들러
// ============================================================

// GET / -> 로그 뷰어
static esp_err_t handleRoot(httpd_req_t *req)
{
    updateActivity();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, HTML_LOG_VIEWER, HTTPD_RESP_USE_STRLEN);
}

// GET /settings -> 설정 페이지
static esp_err_t handleSettingsPage(httpd_req_t *req)
{
    updateActivity();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, HTML_SETTINGS, HTTPD_RESP_USE_STRLEN);
}

// GET /api/settings -> JSON 설정 읽기
static esp_err_t handleGetSettings(httpd_req_t *req)
{
    updateActivity();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "phone", gApp.phoneNumber);
    cJSON_AddNumberToObject(root, "screen_off_min", gApp.screenOffMin);
    cJSON_AddNumberToObject(root, "poweroff_min",   gApp.poweroffMin);
    char *json = cJSON_PrintUnformatted(root);
    if (!json) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON format error");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);

    free(json);
    cJSON_Delete(root);
    return ret;
}

// POST /api/settings -> JSON 설정 저장
static esp_err_t handlePostSettings(httpd_req_t *req)
{
    updateActivity();
    char buf[256];
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *phone = cJSON_GetObjectItem(root, "phone");
    if (phone && cJSON_IsString(phone)) {
        strncpy(gApp.phoneNumber, phone->valuestring, sizeof(gApp.phoneNumber) - 1);
        gApp.phoneNumber[sizeof(gApp.phoneNumber) - 1] = '\0';
        ESP_LOGI(TAG, "Phone number set: %s", gApp.phoneNumber);
    }

    cJSON *screenOff = cJSON_GetObjectItem(root, "screen_off_min");
    if (screenOff && cJSON_IsNumber(screenOff)) {
        int v = (int)screenOff->valuedouble;
        if (v >= 1 && v <= 60) gApp.screenOffMin = (uint16_t)v;
    }

    cJSON *poweroff = cJSON_GetObjectItem(root, "poweroff_min");
    if (poweroff && cJSON_IsNumber(poweroff)) {
        int v = (int)poweroff->valuedouble;
        if (v >= 5 && v <= 180) gApp.poweroffMin = (uint16_t)v;
    }

    cJSON_Delete(root);

    saveSettings();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

// WebSocket /ws/log 핸들러
static esp_err_t handleWsLog(httpd_req_t *req)
{
    updateActivity();
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS client connected");
        return ESP_OK;
    }

    // WS 프레임 수신 (ping/pong은 자동 처리)
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    uint8_t buf[128];
    frame.payload = buf;

    esp_err_t ret = httpd_ws_recv_frame(req, &frame, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "WS recv failed: %s", esp_err_to_name(ret));
    }
    return ESP_OK;
}

// ============================================================
// OTA 핸들러
// ============================================================

// GET /ota -> OTA 업데이트 페이지
static esp_err_t handleOtaPage(httpd_req_t *req)
{
    updateActivity();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, HTML_OTA, HTTPD_RESP_USE_STRLEN);
}

// OTA 완료 후 지연 리부트
static void otaRestartTimerCb(void *arg)
{
    ESP_LOGI(TAG, "Rebooting after OTA...");
    esp_restart();
}

static void scheduleRestart(void)
{
    esp_timer_handle_t timer;
    esp_timer_create_args_t args = {};
    args.callback = otaRestartTimerCb;
    args.name = "ota_restart";
    if (esp_timer_create(&args, &timer) == ESP_OK) {
        esp_timer_start_once(timer, 2000000); // 2초
    }
}

// POST /api/ota/upload -> 펌웨어 바이너리 업로드
static esp_err_t handleOtaUpload(httpd_req_t *req)
{
    updateActivity();
    size_t contentLen = req->content_len;
    if (contentLen == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA upload start: %u bytes", (unsigned)contentLen);

    if (!ota::begin(contentLen)) {
        char resp[128];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", gApp.otaErrorMsg);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }

    // 4KB 수신 버퍼 (동적 할당: WiFi 비활성 시 RAM 반환)
    uint8_t* otaBuf = (uint8_t*)malloc(4096);
    if (!otaBuf) {
        ota::abort();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    size_t remaining = contentLen;
    esp_err_t result = ESP_OK;

    while (remaining > 0) {
        size_t toRead = (remaining < 4096) ? remaining : 4096;
        int received = httpd_req_recv(req, (char *)otaBuf, toRead);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "OTA recv error: %d", received);
            ota::abort();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            free(otaBuf);
            return ESP_FAIL;
        }

        if (!ota::write(otaBuf, received)) {
            char resp[128];
            snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", gApp.otaErrorMsg);
            httpd_resp_set_type(req, "application/json");
            result = httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
            free(otaBuf);
            return result;
        }

        remaining -= received;
    }

    free(otaBuf);

    if (!ota::end()) {
        char resp[128];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", gApp.otaErrorMsg);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(TAG, "OTA complete, scheduling reboot...");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true,\"reboot\":true}", HTTPD_RESP_USE_STRLEN);

    scheduleRestart();
    return ESP_OK;
}

// GET /api/ota/status -> OTA 상태 JSON
static esp_err_t handleOtaStatus(httpd_req_t *req)
{
    updateActivity();

    const char *stateStr;
    switch (gApp.otaState) {
        case OTAState::RECEIVING:  stateStr = "receiving"; break;
        case OTAState::VALIDATING: stateStr = "validating"; break;
        case OTAState::COMPLETE:   stateStr = "complete"; break;
        case OTAState::ERROR:      stateStr = "error"; break;
        default:                   stateStr = "idle"; break;
    }

    char resp[256];
    snprintf(resp, sizeof(resp),
        "{\"version\":\"%s\",\"state\":\"%s\",\"progress\":%.1f,\"received\":%lu,\"total\":%lu,\"error\":\"%s\"}",
        ota::getRunningVersion(),
        stateStr,
        gApp.otaProgress * 100.0f,
        (unsigned long)gApp.otaReceivedBytes,
        (unsigned long)gApp.otaTotalBytes,
        gApp.otaErrorMsg
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// ============================================================
// SD 카드 파일 브라우저
// ============================================================

static const char* humanSize(long bytes, char* buf, int bufLen);  // forward decl

// GET /spiffs → SPIFFS 파일 목록 (읽기 전용, 다운로드 없음)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
static esp_err_t handleSpiffsList(httpd_req_t *req)
{
    updateActivity();
    httpd_resp_set_type(req, "text/html; charset=utf-8");

    char buf[300];

    // HTML 헤더
    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>SPIFFS</title>"
        "<style>"
        "body{font-family:monospace;background:#111;color:#eee;padding:16px;margin:0}"
        "a{color:#4af;text-decoration:none}a:hover{text-decoration:underline}"
        ".nav{margin-bottom:12px}"
        ".row{display:flex;padding:6px 0;border-bottom:1px solid #333}"
        ".row:hover{background:#1a1a2e}"
        ".name{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis}"
        ".size{width:80px;text-align:right;color:#888;flex-shrink:0}"
        ".info{color:#888;margin-bottom:12px}"
        "h2{color:#4af;margin:8px 0 4px}"
        "</style></head><body>");

    // 네비게이션
    httpd_resp_sendstr_chunk(req,
        "<div class='nav'>"
        "<a href='/'>Log</a> &middot; "
        "<a href='/settings'>Settings</a> &middot; "
        "<a href='/ota'>Update</a> &middot; "
        "<a href='/files'>SD</a> &middot; "
        "<b>SPIFFS</b>"
        "</div>");

    httpd_resp_sendstr_chunk(req, "<h2>SPIFFS Files</h2>");

    // SPIFFS 사용량
    size_t total = 0, used = 0;
    if (esp_spiffs_info(nullptr, &total, &used) == ESP_OK) {
        char totalBuf[16], usedBuf[16], freeBuf[16];
        humanSize((long)total, totalBuf, sizeof(totalBuf));
        humanSize((long)used, usedBuf, sizeof(usedBuf));
        humanSize((long)(total - used), freeBuf, sizeof(freeBuf));
        snprintf(buf, sizeof(buf),
                 "<div class='info'>Total: %s &middot; Used: %s &middot; Free: %s (%.0f%%)</div>",
                 totalBuf, usedBuf, freeBuf,
                 100.0 * (total - used) / (total > 0 ? total : 1));
        httpd_resp_sendstr_chunk(req, buf);
    } else {
        httpd_resp_sendstr_chunk(req, "<p style='color:#f44'>SPIFFS not mounted</p></body></html>");
        return httpd_resp_sendstr_chunk(req, nullptr);
    }

    // 파일 목록
    DIR* dir = opendir("/spiffs");
    if (!dir) {
        httpd_resp_sendstr_chunk(req, "<p>Cannot open /spiffs</p></body></html>");
        return httpd_resp_sendstr_chunk(req, nullptr);
    }

    int fileCount = 0;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') continue;

        char entryPath[160];
        snprintf(entryPath, sizeof(entryPath), "/spiffs/%s", entry->d_name);

        struct stat st = {};
        stat(entryPath, &st);

        char szBuf[16];
        humanSize((long)st.st_size, szBuf, sizeof(szBuf));
        snprintf(buf, sizeof(buf),
                 "<div class='row'><span class='name'>%s</span>"
                 "<span class='size'>%s</span></div>",
                 entry->d_name, szBuf);
        httpd_resp_sendstr_chunk(req, buf);
        fileCount++;
    }
    closedir(dir);

    if (fileCount == 0) {
        httpd_resp_sendstr_chunk(req, "<p style='color:#888'>No files</p>");
    }

    httpd_resp_sendstr_chunk(req, "</body></html>");
    return httpd_resp_sendstr_chunk(req, nullptr);
}
#pragma GCC diagnostic pop

// GET /files?d=/path  → SD 카드 폴더 탐색 + 파일 다운로드
// d= 파라미터 없으면 루트(/) 표시, 폴더 클릭으로 탐색, 파일 클릭으로 다운로드
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"

static const char* humanSize(long bytes, char* buf, int bufLen) {
    if (bytes < 1024) {
        snprintf(buf, bufLen, "%ld B", bytes);
    } else if (bytes < 1024 * 1024) {
        snprintf(buf, bufLen, "%.1f KB", bytes / 1024.0);
    } else {
        snprintf(buf, bufLen, "%.1f MB", bytes / (1024.0 * 1024.0));
    }
    return buf;
}

static esp_err_t handleFileList(httpd_req_t *req)
{
    updateActivity();
    httpd_resp_set_type(req, "text/html; charset=utf-8");

    // d= 쿼리 파라미터에서 현재 디렉토리 읽기
    char relDir[128] = "/";
    char qbuf[128] = {};
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        httpd_query_key_value(qbuf, "d", relDir, sizeof(relDir));
    }

    // path traversal 방지
    if (strstr(relDir, "..")) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Forbidden");
        return ESP_FAIL;
    }

    // /sdcard + relDir 조합
    char fullDir[160];
    if (strcmp(relDir, "/") == 0) {
        snprintf(fullDir, sizeof(fullDir), "/sdcard");
    } else {
        snprintf(fullDir, sizeof(fullDir), "/sdcard%s", relDir);
    }

    char buf[300];

    // HTML 헤더
    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>SD Files</title>"
        "<style>"
        "body{font-family:monospace;background:#111;color:#eee;padding:16px;margin:0}"
        "a{color:#4af;text-decoration:none}a:hover{text-decoration:underline}"
        ".nav{margin-bottom:12px}"
        ".row{display:flex;padding:6px 0;border-bottom:1px solid #333}"
        ".row:hover{background:#1a1a2e}"
        ".name{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis}"
        ".size{width:80px;text-align:right;color:#888;flex-shrink:0}"
        ".dir{color:#ffa726}.back{color:#888}"
        "h2{color:#4af;margin:8px 0 4px}"
        "</style></head><body>");

    // 네비게이션
    httpd_resp_sendstr_chunk(req,
        "<div class='nav'>"
        "<a href='/'>Log</a> &middot; "
        "<a href='/settings'>Settings</a> &middot; "
        "<a href='/ota'>Update</a> &middot; "
        "<b>SD</b> &middot; "
        "<a href='/spiffs'>SPIFFS</a>"
        "</div>");

    // 현재 경로 표시
    snprintf(buf, sizeof(buf), "<h2>%s</h2>", relDir);
    httpd_resp_sendstr_chunk(req, buf);

    if (!sdcardIsMounted()) {
        httpd_resp_sendstr_chunk(req, "<p style='color:#f44'>SD card not mounted</p></body></html>");
        return httpd_resp_sendstr_chunk(req, NULL);
    }

    // 상위 폴더 링크 (루트가 아닐 때)
    if (strcmp(relDir, "/") != 0) {
        char parent[128];
        strncpy(parent, relDir, sizeof(parent));
        parent[sizeof(parent) - 1] = '\0';
        char* lastSlash = strrchr(parent, '/');
        if (lastSlash && lastSlash != parent) {
            *lastSlash = '\0';
        } else {
            strcpy(parent, "/");
        }
        snprintf(buf, sizeof(buf),
                 "<div class='row'><span class='name'>"
                 "<a class='back' href='/files?d=%s'>&larr; ..</a>"
                 "</span><span class='size'></span></div>", parent);
        httpd_resp_sendstr_chunk(req, buf);
    }

    DIR* dir = opendir(fullDir);
    if (!dir) {
        httpd_resp_sendstr_chunk(req, "<p>Cannot open directory</p></body></html>");
        return httpd_resp_sendstr_chunk(req, NULL);
    }

    // 먼저 폴더, 그다음 파일 (2-pass)
    // pass 0: 디렉토리, pass 1: 파일
    for (int pass = 0; pass < 2; pass++) {
        rewinddir(dir);
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_name[0] == '.') continue;

            char entryPath[200];
            snprintf(entryPath, sizeof(entryPath), "%s/%s", fullDir, entry->d_name);

            struct stat st = {};
            stat(entryPath, &st);
            bool isDir = S_ISDIR(st.st_mode);

            if ((pass == 0) != isDir) continue;

            if (isDir) {
                // 폴더: 클릭하면 탐색
                char childRel[160];
                if (strcmp(relDir, "/") == 0) {
                    snprintf(childRel, sizeof(childRel), "/%s", entry->d_name);
                } else {
                    snprintf(childRel, sizeof(childRel), "%s/%s", relDir, entry->d_name);
                }
                snprintf(buf, sizeof(buf),
                         "<div class='row'><span class='name'>"
                         "<a class='dir' href='/files?d=%s'>%s/</a>"
                         "</span><span class='size'></span></div>",
                         childRel, entry->d_name);
            } else {
                // 파일: 클릭하면 다운로드
                const char* dlPath = entryPath + 7;  // skip "/sdcard"
                char szBuf[16];
                humanSize((long)st.st_size, szBuf, sizeof(szBuf));
                snprintf(buf, sizeof(buf),
                         "<div class='row'><span class='name'>"
                         "<a href='/api/file?p=%s'>%s</a>"
                         "</span><span class='size'>%s</span></div>",
                         dlPath, entry->d_name, szBuf);
            }
            httpd_resp_sendstr_chunk(req, buf);
        }
    }

    closedir(dir);

    httpd_resp_sendstr_chunk(req, "</body></html>");
    return httpd_resp_sendstr_chunk(req, NULL);
}
#pragma GCC diagnostic pop

// GET /api/file?p=/logs/xxx.log  → 파일 다운로드
static esp_err_t handleFileDownload(httpd_req_t *req)
{
    updateActivity();

    char qbuf[128] = {};
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }
    char pval[100] = {};
    if (httpd_query_key_value(qbuf, "p", pval, sizeof(pval)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing p param");
        return ESP_FAIL;
    }

    // path traversal 방지
    if (strstr(pval, "..")) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Forbidden");
        return ESP_FAIL;
    }

    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "/sdcard%s", pval);

    FILE* f = fopen(fullPath, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/octet-stream");

    const char* fname = strrchr(pval, '/');
    fname = fname ? fname + 1 : pval;
    char cdHeader[128];
    snprintf(cdHeader, sizeof(cdHeader), "attachment; filename=\"%s\"", fname);
    httpd_resp_set_hdr(req, "Content-Disposition", cdHeader);

    char* fileBuf = (char*)malloc(512);
    if (!fileBuf) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    size_t n;
    while ((n = fread(fileBuf, 1, 512, f)) > 0) {
        if (httpd_resp_send_chunk(req, fileBuf, (ssize_t)n) != ESP_OK) break;
    }
    fclose(f);
    free(fileBuf);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// Favicon (SVG 스톱워치 아이콘)
static const char FAVICON_SVG[] =
    "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'>"
    "<circle cx='16' cy='18' r='11' fill='none' stroke='#e94560' stroke-width='2.5'/>"
    "<line x1='16' y1='18' x2='16' y2='10' stroke='#e94560' stroke-width='2.5' stroke-linecap='round'/>"
    "<line x1='16' y1='18' x2='22' y2='18' stroke='#e94560' stroke-width='2' stroke-linecap='round'/>"
    "<rect x='14' y='3' width='4' height='3' rx='1' fill='#e94560'/>"
    "<line x1='24' y1='9' x2='26' y2='7' stroke='#e94560' stroke-width='2' stroke-linecap='round'/>"
    "</svg>";

static esp_err_t handleFavicon(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=604800");
    return httpd_resp_send(req, FAVICON_SVG, HTTPD_RESP_USE_STRLEN);
}

// ============================================================
// HTTP 서버 초기화
// ============================================================

static void initHttpServer(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = ENABLE_LIVE_LOG_WS ? 15 : 14;
    config.max_open_sockets = 2;  // 내부 RAM 절약
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 4096;

    ESP_LOGI(TAG, "HTTP pre-start: int_free=%lu largest=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    if (httpd_start(&s_httpServer, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed (int_free=%lu largest=%lu)",
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        return;
    }

    // 로그 뷰어 (메인 페이지)
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handleRoot,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_root);

    // 설정 페이지
    httpd_uri_t uri_settings = {
        .uri = "/settings",
        .method = HTTP_GET,
        .handler = handleSettingsPage,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_settings);

    // API: 설정 읽기
    httpd_uri_t uri_get_settings = {
        .uri = "/api/settings",
        .method = HTTP_GET,
        .handler = handleGetSettings,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_get_settings);

    // API: 설정 저장
    httpd_uri_t uri_post_settings = {
        .uri = "/api/settings",
        .method = HTTP_POST,
        .handler = handlePostSettings,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_post_settings);

    // WebSocket: 로그 스트리밍
    if (ENABLE_LIVE_LOG_WS) {
        httpd_uri_t uri_ws_log = {
            .uri = "/ws/log",
            .method = HTTP_GET,
            .handler = handleWsLog,
            .user_ctx = nullptr,
            .is_websocket = true,
            .handle_ws_control_frames = false,
            .supported_subprotocol = nullptr
        };
        httpd_register_uri_handler(s_httpServer, &uri_ws_log);
    }

    // Favicon
    httpd_uri_t uri_favicon = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = handleFavicon,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_favicon);

    // OTA 업데이트 페이지
    httpd_uri_t uri_ota = {
        .uri = "/ota",
        .method = HTTP_GET,
        .handler = handleOtaPage,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_ota);

    // OTA 펌웨어 업로드
    httpd_uri_t uri_ota_upload = {
        .uri = "/api/ota/upload",
        .method = HTTP_POST,
        .handler = handleOtaUpload,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_ota_upload);

    // OTA 상태 JSON
    httpd_uri_t uri_ota_status = {
        .uri = "/api/ota/status",
        .method = HTTP_GET,
        .handler = handleOtaStatus,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_ota_status);

    // SPIFFS 파일 브라우저
    httpd_uri_t uri_spiffs = {
        .uri = "/spiffs",
        .method = HTTP_GET,
        .handler = handleSpiffsList,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_spiffs);

    // SD 카드 파일 브라우저
    httpd_uri_t uri_files = {
        .uri = "/files",
        .method = HTTP_GET,
        .handler = handleFileList,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_files);

    httpd_uri_t uri_file_download = {
        .uri = "/api/file",
        .method = HTTP_GET,
        .handler = handleFileDownload,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_file_download);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
}

// ============================================================
// SoftAP 초기화 (1회) / 시작 / 중지
// ============================================================

static void wifiEventHandler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Client connected, AID=%d (int_free=%lu int_largest=%lu)",
                 event->aid,
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        updateActivity();
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Client disconnected, AID=%d", event->aid);
    }
}

static void ipEventHandler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *)event_data;
        ESP_LOGI(TAG, "DHCP assigned IP: " IPSTR " to client", IP2STR(&event->ip));
    }
}

// 1회 초기화: netif + event loop + wifi driver
static void initWifiDriver(void)
{
    if (s_wifiInitialized) return;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_apNetif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &ipEventHandler, NULL, NULL));

    s_wifiInitialized = true;
}

static void startSoftAP(void)
{
    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.ap.ssid, WIFI_AP_SSID);
    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    strcpy((char *)wifi_config.ap.password, WIFI_AP_PASSWORD);
    wifi_config.ap.channel = 1;
    // Single-user OTA portal: allow only one station to minimize AP footprint.
    wifi_config.ap.max_connection = 1;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(pdMS_TO_TICKS(25));

    // DHCP 서버 명시적 재시작 (부팅 시 auto-start 후 응답 안 하는 문제 대응)
    if (s_apNetif) {
        esp_netif_dhcps_stop(s_apNetif);
        esp_err_t err = esp_netif_dhcps_start(s_apNetif);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "DHCP server restart failed: %s", esp_err_to_name(err));
        } else {
            esp_netif_ip_info_t ip_info;
            esp_netif_get_ip_info(s_apNetif, &ip_info);
            ESP_LOGI(TAG, "DHCP server restarted (IP: " IPSTR ")", IP2STR(&ip_info.ip));
        }
    }

    // WiFi 전력절약 끄기 (DHCP 패킷 드롭 방지)
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "SoftAP started: SSID=" WIFI_AP_SSID " (WPA2), int_free=%lu, int_largest=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

// ============================================================
// 설정 저장/로드 (SPIFFS JSON)
// ============================================================

void loadSettings(void)
{
    FILE *f = fopen(SETTINGS_PATH, "r");
    if (!f) {
        ESP_LOGI(TAG, "No settings file, using defaults");
        return;
    }

    char buf[256];
    size_t len = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "Settings JSON parse error");
        return;
    }

    cJSON *phone = cJSON_GetObjectItem(root, "phone");
    if (phone && cJSON_IsString(phone)) {
        strncpy(gApp.phoneNumber, phone->valuestring, sizeof(gApp.phoneNumber) - 1);
        gApp.phoneNumber[sizeof(gApp.phoneNumber) - 1] = '\0';
    }

    cJSON *screenOff = cJSON_GetObjectItem(root, "screen_off_min");
    if (screenOff && cJSON_IsNumber(screenOff)) {
        int v = (int)screenOff->valuedouble;
        if (v >= 1 && v <= 60) gApp.screenOffMin = (uint16_t)v;
    }

    cJSON *poweroff = cJSON_GetObjectItem(root, "poweroff_min");
    if (poweroff && cJSON_IsNumber(poweroff)) {
        int v = (int)poweroff->valuedouble;
        if (v >= 5 && v <= 180) gApp.poweroffMin = (uint16_t)v;
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Settings loaded: phone=%s screen_off=%dm poweroff=%dm",
             gApp.phoneNumber, gApp.screenOffMin, gApp.poweroffMin);
}

void saveSettings(void)
{
    // config 디렉토리 생성 (없으면)
    mkdir("/spiffs/config", 0755);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "phone", gApp.phoneNumber);
    cJSON_AddNumberToObject(root, "screen_off_min", gApp.screenOffMin);
    cJSON_AddNumberToObject(root, "poweroff_min",   gApp.poweroffMin);
    char *json = cJSON_PrintUnformatted(root);
    if (!json) {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Failed to format settings JSON");
        return;
    }

    FILE *f = fopen(SETTINGS_PATH, "w");
    if (f) {
        fwrite(json, 1, strlen(json), f);
        fclose(f);
        ESP_LOGI(TAG, "Settings saved");
    } else {
        ESP_LOGE(TAG, "Failed to write settings");
    }

    free(json);
    cJSON_Delete(root);
}

// ============================================================
// Public API
// ============================================================

void initWifiPortal(void)
{
    ESP_LOGI(TAG, "Initializing WiFi portal (radio OFF)...");

    // 로그 링 버퍼 초기화
    s_logRing.init();

    // WiFi 드라이버 1회 초기화 (라디오 시작 안 함)
    initWifiDriver();

    ESP_LOGI(TAG, "WiFi portal initialized (use 'w' to start)");
}

void startWifiPortal(void)
{
    if (s_wifiActive) {
        ESP_LOGW(TAG, "WiFi portal already active");
        return;
    }
    if (!s_wifiInitialized) {
        initWifiDriver();
    }

    ESP_LOGI(TAG, "Starting WiFi portal...");

    // SoftAP 시작
    startSoftAP();

    // HTTP + WebSocket 서버
    initHttpServer();

    // 로그 후킹 (esp_log 출력을 링 버퍼로 복사)
    if (!s_origVprintf) {
        s_origVprintf = esp_log_set_vprintf(logHookVprintf);
    }

    // 로그 플러시 + 유휴 체크 타이머 (200ms 주기)
    if (!s_logFlushTimer) {
        s_logFlushTimer = xTimerCreate("log_flush", pdMS_TO_TICKS(200),
                                        pdTRUE, nullptr, logFlushTimerCallback);
    }
    xTimerStart(s_logFlushTimer, 0);

    // 활동 타임스탬프 초기화
    updateActivity();

    s_wifiActive = true;
    ESP_LOGI(TAG, "WiFi portal ready: http://" WIFI_AP_IP_STR);
}

void stopWifiPortal(void)
{
    if (!s_wifiActive) {
        return;
    }

    ESP_LOGI(TAG, "Stopping WiFi portal...");

    // 타이머 정지
    if (s_logFlushTimer) {
        xTimerStop(s_logFlushTimer, portMAX_DELAY);
    }

    // 로그 후킹 복원
    if (s_origVprintf) {
        esp_log_set_vprintf(s_origVprintf);
        s_origVprintf = nullptr;
    }

    // HTTP 서버 정지
    if (s_httpServer) {
        httpd_stop(s_httpServer);
        s_httpServer = nullptr;
    }

    // WiFi 라디오 정지
    esp_wifi_stop();

    s_wifiActive = false;
    s_lastActivityUs = 0;

    ESP_LOGI(TAG, "WiFi portal stopped (power saved)");
}

void toggleWifiPortal(void)
{
    if (s_wifiActive) {
        stopWifiPortal();
    } else {
        startWifiPortal();
    }
}

bool isWifiPortalActive(void)
{
    return s_wifiActive;
}
