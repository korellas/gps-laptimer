/**
 * @file wifi_portal.cpp
 * @brief WiFi Captive Portal — SoftAP + DNS redirect + HTTP + WebSocket log
 *
 * - SoftAP SSID: "LAPTIMER" (open, no password)
 * - DNS: 모든 도메인 -> 192.168.4.1 (캡티브 포털 트리거)
 * - HTTP GET /              -> 로그 뷰어
 * - HTTP GET /settings      -> 설정 페이지
 * - HTTP GET /ota           -> OTA 업데이트 페이지
 * - HTTP GET /api/settings  -> JSON 설정 읽기
 * - HTTP POST /api/settings -> JSON 설정 저장
 * - HTTP POST /api/ota/upload -> 펌웨어 바이너리 업로드
 * - HTTP GET /api/ota/status  -> OTA 상태 JSON
 * - WS /ws/log              -> WebSocket 로그 스트리밍
 * - 404                     -> 302 리다이렉트 -> / (캡티브 포털)
 *
 * WiFi는 기본 OFF. startWifiPortal()로 시작, 5분 무접속 시 자동 OFF.
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

#include "dns_server.h"
#include "types.h"
#include "wifi_portal.h"
#include "log_ringbuffer.h"
#include "wifi_portal_html.h"
#include "ota_manager.h"

static const char *TAG = "wifi_portal";

// ============================================================
// 전역 상태
// ============================================================

static httpd_handle_t s_httpServer = nullptr;
static dns_server_handle_t s_dnsHandle = nullptr;
static LogRingBuffer s_logRing;
static vprintf_like_t s_origVprintf = nullptr;
static TimerHandle_t s_logFlushTimer = nullptr;

static constexpr const char *SETTINGS_PATH = "/spiffs/config/settings.json";
static constexpr int WS_MAX_CLIENTS = 4;
// Keep disabled by default to preserve internal RAM for SoftAP auth/DHCP.
static constexpr bool ENABLE_LIVE_LOG_WS = false;

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
    char *json = cJSON_PrintUnformatted(root);

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

    // 4KB 수신 버퍼 (static: 스택 사용 최소화)
    static uint8_t otaBuf[4096];
    size_t remaining = contentLen;

    while (remaining > 0) {
        size_t toRead = (remaining < sizeof(otaBuf)) ? remaining : sizeof(otaBuf);
        int received = httpd_req_recv(req, (char *)otaBuf, toRead);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "OTA recv error: %d", received);
            ota::abort();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }

        if (!ota::write(otaBuf, received)) {
            char resp[128];
            snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", gApp.otaErrorMsg);
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        }

        remaining -= received;
    }

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

// 404 -> 302 리다이렉트 / (캡티브 포털)
static esp_err_t handleNotFound(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
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

// 캡티브 포털 감지용 (Android/iOS/Windows)
static esp_err_t handleCaptiveCheck(httpd_req_t *req)
{
    updateActivity();
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" WIFI_AP_IP_STR "/");
    return httpd_resp_send(req, NULL, 0);
}

// ============================================================
// HTTP 서버 초기화
// ============================================================

static void initHttpServer(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = ENABLE_LIVE_LOG_WS ? 12 : 11;
    config.max_open_sockets = 2;  // 3→2: save ~11KB internal RAM for WiFi DMA TX
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

    // 캡티브 포털 감지 URL (Android)
    httpd_uri_t uri_generate_204 = {
        .uri = "/generate_204",
        .method = HTTP_GET,
        .handler = handleCaptiveCheck,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_generate_204);

    // 캡티브 포털 감지 URL (Apple)
    httpd_uri_t uri_hotspot = {
        .uri = "/hotspot-detect.html",
        .method = HTTP_GET,
        .handler = handleCaptiveCheck,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_hotspot);

    // 캡티브 포털 감지 URL (Windows)
    httpd_uri_t uri_connecttest = {
        .uri = "/connecttest.txt",
        .method = HTTP_GET,
        .handler = handleCaptiveCheck,
        .user_ctx = nullptr,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(s_httpServer, &uri_connecttest);

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

    // 404 -> 리다이렉트 (나머지 모든 요청)
    httpd_register_err_handler(s_httpServer, HTTPD_404_NOT_FOUND, handleNotFound);

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
    wifi_config.ap.channel = 1;
    // Single-user OTA portal: allow only one station to minimize AP footprint.
    wifi_config.ap.max_connection = 1;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;

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

    ESP_LOGI(TAG, "SoftAP started: SSID=" WIFI_AP_SSID " (open), int_free=%lu, int_largest=%lu",
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

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Settings loaded: phone=%s", gApp.phoneNumber);
}

void saveSettings(void)
{
    // config 디렉토리 생성 (없으면)
    mkdir("/spiffs/config", 0755);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "phone", gApp.phoneNumber);
    char *json = cJSON_PrintUnformatted(root);

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

    // HTTP + WebSocket 서버 (DNS보다 먼저 — 내부 RAM 여유 확보)
    initHttpServer();

    // DNS 리다이렉트 시작 (모든 도메인 -> AP IP)
    dns_server_config_t dns_config = DNS_SERVER_CONFIG_SINGLE("*", "WIFI_AP_DEF");
    s_dnsHandle = start_dns_server(&dns_config);
    ESP_LOGI(TAG, "DNS redirect server started");

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

    // DNS 서버 정지
    if (s_dnsHandle) {
        stop_dns_server(s_dnsHandle);
        s_dnsHandle = nullptr;
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
