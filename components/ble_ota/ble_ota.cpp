/**
 * @file ble_ota.cpp
 * @brief BLE OTA firmware update service using NimBLE
 *
 * GATT Service (UUID: 0xFFE0):
 *   - Control (0xFFE1): Write — START/END/ABORT/VERSION commands
 *   - Data    (0xFFE2): Write No Response — firmware chunks (max 512B)
 *   - Status  (0xFFE3): Read + Notify — state/progress/version
 *
 * On-demand: startBleOta() → NimBLE init + advertise,
 * stopBleOta() → shutdown. WiFi is stopped during BLE OTA.
 */

#include "ble_ota.h"
#include "types.h"
#include "ota_manager.h"
#include "wifi_portal.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <cstring>

static const char *TAG = "BLE_OTA";

// ============================================================
// State
// ============================================================

static bool s_active = false;
static uint16_t s_connHandle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_statusValHandle = 0;
static uint8_t s_addrType;
static int s_lastNotifyPct = -1;

// Forward declarations
static void startAdvertising(void);
static void sendStatusNotify(void);
static int bleGapEvent(struct ble_gap_event *event, void *arg);

// ============================================================
// Delayed restart after OTA
// ============================================================

static void restartTimerCb(void *arg)
{
    ESP_LOGI(TAG, "Rebooting after BLE OTA...");
    esp_restart();
}

static void scheduleRestart(void)
{
    esp_timer_handle_t t;
    esp_timer_create_args_t a = {};
    a.callback = restartTimerCb;
    a.name = "ble_ota_rst";
    if (esp_timer_create(&a, &t) == ESP_OK) {
        esp_timer_start_once(t, 2000000); // 2s
    }
}

// ============================================================
// Status payload builder (19 bytes)
// [0] state  [1] progress%  [2] error  [3..18] version
// ============================================================

static void buildStatusPayload(uint8_t *payload)
{
    memset(payload, 0, 19);
    switch (gApp.otaState) {
        case OTAState::IDLE:       payload[0] = 0; break;
        case OTAState::RECEIVING:  payload[0] = 1; break;
        case OTAState::VALIDATING: payload[0] = 2; break;
        case OTAState::COMPLETE:   payload[0] = 3; break;
        case OTAState::ERROR:      payload[0] = 4; break;
    }
    payload[1] = (uint8_t)(gApp.otaProgress * 100);
    payload[2] = (gApp.otaState == OTAState::ERROR) ? 0xFF : 0;
    strncpy((char *)&payload[3], ota::getRunningVersion(), 16);
}

// ============================================================
// GATT access callbacks
// ============================================================

// OTA Control (0xFFE1) — Write
static int onControlAccess(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t cmd;
    os_mbuf_copydata(ctxt->om, 0, 1, &cmd);

    switch (cmd) {
    case 0x01: { // START — [cmd(1) + size(4)]
        if (len < 5) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint32_t imageSize;
        os_mbuf_copydata(ctxt->om, 1, 4, &imageSize);
        ESP_LOGI(TAG, "CMD START: %lu bytes", (unsigned long)imageSize);
        s_lastNotifyPct = -1;
        if (!ota::begin(imageSize)) {
            sendStatusNotify();
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        sendStatusNotify();
        break;
    }
    case 0x02: // END
        ESP_LOGI(TAG, "CMD END");
        if (!ota::end()) {
            sendStatusNotify();
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        sendStatusNotify();
        scheduleRestart();
        break;
    case 0x03: // ABORT
        ESP_LOGI(TAG, "CMD ABORT");
        ota::abort();
        sendStatusNotify();
        break;
    case 0x04: // VERSION
        sendStatusNotify();
        break;
    default:
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    return 0;
}

// OTA Data (0xFFE2) — Write Without Response
static int onDataAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    static uint8_t dataBuf[512];
    if (len > sizeof(dataBuf)) len = sizeof(dataBuf);
    os_mbuf_copydata(ctxt->om, 0, len, dataBuf);

    if (!ota::write(dataBuf, len)) {
        sendStatusNotify();
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    // Notify progress every 5%
    int pct = (int)(gApp.otaProgress * 100);
    if (pct / 5 != s_lastNotifyPct / 5) {
        s_lastNotifyPct = pct;
        sendStatusNotify();
    }

    return 0;
}

// OTA Status (0xFFE3) — Read
static int onStatusAccess(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return 0;

    uint8_t payload[19];
    buildStatusPayload(payload);
    os_mbuf_append(ctxt->om, payload, sizeof(payload));
    return 0;
}

// ============================================================
// Status notification
// ============================================================

static void sendStatusNotify(void)
{
    if (s_connHandle == BLE_HS_CONN_HANDLE_NONE || s_statusValHandle == 0) return;

    uint8_t payload[19];
    buildStatusPayload(payload);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
    if (om) {
        ble_gatts_notify_custom(s_connHandle, s_statusValHandle, om);
    }
}

// ============================================================
// GATT service definition
// ============================================================

static const ble_uuid16_t s_svcUuid  = BLE_UUID16_INIT(0xFFE0);
static const ble_uuid16_t s_ctrlUuid = BLE_UUID16_INIT(0xFFE1);
static const ble_uuid16_t s_dataUuid = BLE_UUID16_INIT(0xFFE2);
static const ble_uuid16_t s_statUuid = BLE_UUID16_INIT(0xFFE3);

static struct ble_gatt_chr_def s_otaChrs[] = {
    {
        .uuid = &s_ctrlUuid.u,
        .access_cb = onControlAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE,
        .min_key_size = 0,
        .val_handle = nullptr,
        .cpfd = nullptr,
    },
    {
        .uuid = &s_dataUuid.u,
        .access_cb = onDataAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = nullptr,
        .cpfd = nullptr,
    },
    {
        .uuid = &s_statUuid.u,
        .access_cb = onStatusAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &s_statusValHandle,
        .cpfd = nullptr,
    },
    { },  // terminator
};

static const struct ble_gatt_svc_def s_gattSvcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svcUuid.u,
        .includes = nullptr,
        .characteristics = s_otaChrs,
    },
    { },  // terminator
};

// ============================================================
// GAP event handler
// ============================================================

static int bleGapEvent(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connHandle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, handle=%d", s_connHandle);
            // MTU 교환은 클라이언트(Web Bluetooth)가 시작
            // 빠른 연결 인터벌 요청 (7.5ms)
            struct ble_gap_upd_params upd = {};
            upd.itvl_min = 6;              // 7.5ms (1.25ms 단위)
            upd.itvl_max = 6;
            upd.latency = 0;
            upd.supervision_timeout = 500;  // 5s (10ms 단위)
            ble_gap_update_params(s_connHandle, &upd);
        } else {
            startAdvertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason=%d", event->disconnect.reason);
        s_connHandle = BLE_HS_CONN_HANDLE_NONE;
        if (ota::isInProgress()) {
            ESP_LOGW(TAG, "OTA aborted (disconnect)");
            ota::abort();
        }
        if (s_active) {
            startAdvertising();
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU: %d", event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

// ============================================================
// Advertising
// ============================================================

static void startAdvertising(void)
{
    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params params = {};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_start(s_addrType, NULL, BLE_HS_FOREVER,
                                &params, bleGapEvent, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Advertising start failed: %d", rc);
    }
}

// ============================================================
// NimBLE host callbacks
// ============================================================

static void onSync(void)
{
    ble_hs_id_infer_auto(0, &s_addrType);
    ESP_LOGI(TAG, "NimBLE synced, advertising...");
    startAdvertising();
}

static void onReset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason);
}

static void bleHostTask(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ============================================================
// Public API
// ============================================================

void startBleOta(void)
{
    if (s_active) {
        ESP_LOGW(TAG, "BLE OTA already active");
        return;
    }

    // WiFi/BLE 라디오 배타성 — WiFi 중지
    if (isWifiPortalActive()) {
        stopWifiPortal();
        ESP_LOGI(TAG, "WiFi stopped for BLE OTA");
    }

    // NimBLE 초기화
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return;
    }

    // 높은 MTU 요청 (512B 데이터 전송용)
    ble_att_set_preferred_mtu(517);

    // GAP/GATT 서비스 초기화
    ble_svc_gap_device_name_set("LAPTIMER-OTA");
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // OTA GATT 서비스 등록
    ble_gatts_count_cfg(s_gattSvcs);
    ble_gatts_add_svcs(s_gattSvcs);

    // Host 콜백
    ble_hs_cfg.sync_cb = onSync;
    ble_hs_cfg.reset_cb = onReset;

    // NimBLE host task 시작
    nimble_port_freertos_init(bleHostTask);

    s_active = true;
    s_connHandle = BLE_HS_CONN_HANDLE_NONE;
    ESP_LOGI(TAG, "BLE OTA started, advertising as LAPTIMER-OTA");
}

void stopBleOta(void)
{
    if (!s_active) return;

    if (ota::isInProgress()) {
        ota::abort();
    }

    int rc = nimble_port_stop();
    if (rc == 0) {
        nimble_port_deinit();
    }

    s_active = false;
    s_connHandle = BLE_HS_CONN_HANDLE_NONE;
    ESP_LOGI(TAG, "BLE OTA stopped");
}

bool isBleOtaActive(void)
{
    return s_active;
}
