/**
 * @file ble_ota.cpp
 * @brief BLE OTA firmware update service using NimBLE
 *
 * GATT Service (UUID: 0xFFE0):
 *   - Control (0xFFE1): Write ??START/END/ABORT/VERSION commands
 *   - Data    (0xFFE2): Write No Response ??firmware chunks (max 512B)
 *   - Status  (0xFFE3): Read + Notify ??state/progress/version
 *
 * On-demand: startBleOta() ??NimBLE init + advertise,
 * stopBleOta() ??shutdown. WiFi ?댁젣???몄텧??梨낆엫.
 */

#include "ble_ota.h"
#include "types.h"
#include "ota_manager.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <cstring>
#include <cstdio>

static const char *TAG = "BLE_OTA";

// ============================================================
// State
// ============================================================

static bool s_active = false;
static bool s_starting = false;
static bool s_advertising = false;
static volatile bool s_hostTaskRunning = false;
static TaskHandle_t s_hostTaskHandle = nullptr;
static StackType_t *s_hostTaskStack = nullptr;
static StaticTask_t *s_hostTaskTcb = nullptr;
static uint32_t s_hostTaskStackWords = 0;
static volatile bool s_synced = false;
static uint16_t s_connHandle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_statusValHandle = 0;
static uint8_t s_addrType;
static int s_lastNotifyPct = -1;

static constexpr TickType_t HOST_TASK_STOP_TIMEOUT_TICKS  = pdMS_TO_TICKS(1000);
static constexpr TickType_t HS_SYNC_TIMEOUT_TICKS         = pdMS_TO_TICKS(4000);

// Forward declarations
static void startAdvertising(void);
static void sendStatusNotify(void);
static int bleGapEvent(struct ble_gap_event *event, void *arg);
static bool waitForTrue(volatile bool &flag, TickType_t timeoutTicks);
static bool waitForFalse(volatile bool &flag, TickType_t timeoutTicks);
static esp_err_t startNimbleHostTask(void);
static void freeNimbleHostTaskBuffers(void);
static void forceBtControllerIdle(void);

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

static int onControlAccess(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t cmd;
    os_mbuf_copydata(ctxt->om, 0, 1, &cmd);

    switch (cmd) {
    case 0x01: { // START
        if (len < 5) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint32_t imageSize;
        os_mbuf_copydata(ctxt->om, 1, 4, &imageSize);
        ESP_LOGI(TAG, "CMD START: %lu bytes", (unsigned long)imageSize);

        // Guard against OTA begin on external-RAM task stack.
        // This can trigger cache freeze asserts in esp_ota_begin().
        uint32_t stackProbe = 0;
        if (!esp_ptr_internal(&stackProbe)) {
            ESP_LOGE(TAG, "CMD START rejected: callback stack is not in internal RAM");
            snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "BLE host stack must be internal RAM");
            gApp.otaState = OTAState::ERROR;
            sendStatusNotify();
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }

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

    int pct = (int)(gApp.otaProgress * 100);
    if (pct / 5 != s_lastNotifyPct / 5) {
        s_lastNotifyPct = pct;
        sendStatusNotify();
    }

    return 0;
}

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
            struct ble_gap_upd_params upd = {};
            upd.itvl_min = 6;
            upd.itvl_max = 6;
            upd.latency = 0;
            upd.supervision_timeout = 500;
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

    static ble_uuid16_t advSvcUuids[] = { BLE_UUID16_INIT(0xFFE0) };
    fields.uuids16 = advSvcUuids;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed: %d", rc);
        s_advertising = false;
        return;
    }

    struct ble_gap_adv_params params = {};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_addrType, NULL, BLE_HS_FOREVER,
                            &params, bleGapEvent, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        s_advertising = false;
    } else {
        s_advertising = true;
        ESP_LOGI(TAG, "Advertising OK as '%s'", name);
    }
}

// ============================================================
// NimBLE host callbacks
// ============================================================

static void onSync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_addrType);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    s_synced = true;
    ESP_LOGI(TAG, "NimBLE synced (addrType=%d)", s_addrType);
    startAdvertising();
}

static void onReset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason);
    s_synced = false;
    s_advertising = false;
}

static void bleHostTask(void *param)
{
    s_hostTaskHandle = xTaskGetCurrentTaskHandle();
    s_hostTaskRunning = true;
    ESP_LOGI(TAG, "Host task started (free stack: %u)",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    nimble_port_run();
    ESP_LOGI(TAG, "Host task exiting");
    s_hostTaskRunning = false;
    s_hostTaskHandle = nullptr;
    vTaskDelete(NULL);
}

static bool waitForTrue(volatile bool &flag, TickType_t timeoutTicks)
{
    TickType_t start = xTaskGetTickCount();
    while (!flag) {
        if ((xTaskGetTickCount() - start) >= timeoutTicks) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

static bool waitForFalse(volatile bool &flag, TickType_t timeoutTicks)
{
    TickType_t start = xTaskGetTickCount();
    while (flag) {
        if ((xTaskGetTickCount() - start) >= timeoutTicks) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

static esp_err_t startNimbleHostTask(void)
{
    const BaseType_t preferredCore = CONFIG_BT_NIMBLE_PINNED_TO_CORE;
    const uint32_t stackCandidates[] = {
        (uint32_t)CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE,
        6144,
        4096,
    };

    auto tryCreateStatic = [&](BaseType_t core, uint32_t stackBytes) -> esp_err_t {
        const uint32_t stackWords = (stackBytes + sizeof(StackType_t) - 1U) / sizeof(StackType_t);
        if (stackWords == 0) {
            return ESP_ERR_INVALID_ARG;
        }

        StackType_t *stackMem = (StackType_t *)heap_caps_malloc(
            stackWords * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (stackMem == nullptr) {
            ESP_LOGW(TAG, "nimble_host stack alloc failed (core=%ld, stack=%lu, int_largest=%lu)",
                     (long)core, (unsigned long)stackBytes,
                     (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            return ESP_ERR_NO_MEM;
        }

        StaticTask_t *tcbMem = (StaticTask_t *)heap_caps_malloc(
            sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (tcbMem == nullptr) {
            heap_caps_free(stackMem);
            ESP_LOGW(TAG, "nimble_host TCB alloc failed (core=%ld)", (long)core);
            return ESP_ERR_NO_MEM;
        }

        TaskHandle_t h = xTaskCreateStaticPinnedToCore(
            bleHostTask,
            "nimble_host",
            stackWords,
            NULL,
            (configMAX_PRIORITIES - 4),
            stackMem,
            tcbMem,
            core);
        if (h == nullptr) {
            heap_caps_free(tcbMem);
            heap_caps_free(stackMem);
            ESP_LOGW(TAG, "nimble_host static create failed (core=%ld, stack=%lu)",
                     (long)core, (unsigned long)stackBytes);
            return ESP_ERR_NO_MEM;
        }

        s_hostTaskHandle = h;
        s_hostTaskStack = stackMem;
        s_hostTaskTcb = tcbMem;
        s_hostTaskStackWords = stackWords;
        ESP_LOGI(TAG, "nimble_host created (core=%ld, stack=%lu bytes, internal dynamic-static)",
                 (long)core, (unsigned long)(stackWords * sizeof(StackType_t)));
        ESP_LOGI(TAG, "nimble_host stack internal=%d", (int)esp_ptr_internal(s_hostTaskStack));
        return ESP_OK;
    };

    for (uint32_t stackBytes : stackCandidates) {
        if (stackBytes == 0) {
            continue;
        }

        esp_err_t err = tryCreateStatic(preferredCore, stackBytes);
        if (err == ESP_OK) {
            return ESP_OK;
        }

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
        const BaseType_t altCore = (preferredCore == 0) ? 1 : 0;
        ESP_LOGW(TAG, "Retrying nimble_host on alternate core=%ld (stack=%lu)",
                 (long)altCore, (unsigned long)stackBytes);
        err = tryCreateStatic(altCore, stackBytes);
        if (err == ESP_OK) {
            return ESP_OK;
        }
#endif
    }

    ESP_LOGE(TAG, "Failed to create nimble_host task (preferred_core=%d, int_largest=%lu, spiram_largest=%lu)",
             preferredCore,
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    return ESP_ERR_NO_MEM;
}

static void freeNimbleHostTaskBuffers(void)
{
    if (s_hostTaskTcb != nullptr) {
        heap_caps_free(s_hostTaskTcb);
        s_hostTaskTcb = nullptr;
    }
    if (s_hostTaskStack != nullptr) {
        heap_caps_free(s_hostTaskStack);
        s_hostTaskStack = nullptr;
    }
    s_hostTaskStackWords = 0;
}

static void forceBtControllerIdle(void)
{
    esp_bt_controller_status_t status = esp_bt_controller_get_status();
    if (status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_err_t e = esp_bt_controller_disable();
        ESP_LOGI(TAG, "BT controller disable(forced): %s", esp_err_to_name(e));
    }

    status = esp_bt_controller_get_status();
    if (status == ESP_BT_CONTROLLER_STATUS_INITED) {
        esp_err_t e = esp_bt_controller_deinit();
        ESP_LOGI(TAG, "BT controller deinit(forced): %s", esp_err_to_name(e));
    }
}

// ============================================================
// Public API
// ============================================================

esp_err_t startBleOta(void)
{
    if (s_starting) {
        ESP_LOGW(TAG, "Start already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_active) {
        esp_bt_controller_status_t status = esp_bt_controller_get_status();
        if (!s_hostTaskRunning && status == ESP_BT_CONTROLLER_STATUS_IDLE) {
            ESP_LOGW(TAG, "Clearing stale active state");
            s_active = false;
            s_advertising = false;
            s_synced = false;
            s_hostTaskHandle = nullptr;
            freeNimbleHostTaskBuffers();
        } else {
        ESP_LOGW(TAG, "Already active");
        return ESP_ERR_INVALID_STATE;
        }
    }

    s_starting = true;
    s_hostTaskRunning = false;
    s_hostTaskHandle = nullptr;
    freeNimbleHostTaskBuffers();
    s_synced = false;
    s_advertising = false;
    s_connHandle = BLE_HS_CONN_HANDLE_NONE;

    ESP_LOGI(TAG, "=== BLE OTA init ===");
    ESP_LOGI(TAG, "Heap: free=%lu largest=%lu int_free=%lu int_largest=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_LOGW(TAG, "BT controller not idle before init; forcing cleanup");
        forceBtControllerIdle();
    }

    // 1. NimBLE ?ы듃 珥덇린??(BT 而⑦듃濡ㅻ윭 + HCI + host)
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init FAILED: %s", esp_err_to_name(ret));
        s_starting = false;
        return ret;
    }
    ESP_LOGI(TAG, "[1] nimble_port_init OK (heap: %lu)",
             (unsigned long)esp_get_free_heap_size());

    // 2. Host 肄쒕갚 ?ㅼ젙 (?쒕퉬??init ????ESP-IDF bleprph ?⑦꽩)
    ble_hs_cfg.sync_cb = onSync;
    ble_hs_cfg.reset_cb = onReset;

    // 3. MTU
    ble_att_set_preferred_mtu(517);

    // 4. GAP/GATT services
    int rc = ble_svc_gap_device_name_set("LAPTIMER-OTA");
    if (rc != 0) {
        ESP_LOGE(TAG, "device_name_set failed: %d", rc);
        (void)nimble_port_deinit();
        forceBtControllerIdle();
        s_starting = false;
        return ESP_FAIL;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // 5. OTA GATT ?쒕퉬???깅줉
    rc = ble_gatts_count_cfg(s_gattSvcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_count_cfg failed: %d", rc);
        (void)nimble_port_deinit();
        forceBtControllerIdle();
        s_starting = false;
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_gattSvcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_add_svcs failed: %d", rc);
        (void)nimble_port_deinit();
        forceBtControllerIdle();
        s_starting = false;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[2] GATT services OK");

    // 6. NimBLE host task ?쒖옉
    ESP_LOGI(TAG, "[3] Starting host task (heap: %lu)...",
             (unsigned long)esp_get_free_heap_size());
    ret = startNimbleHostTask();
    if (ret != ESP_OK) {
        (void)nimble_port_deinit();
        freeNimbleHostTaskBuffers();
        forceBtControllerIdle();
        s_starting = false;
        return ret;
    }

    s_active = true;

    if (!waitForTrue(s_synced, HS_SYNC_TIMEOUT_TICKS)) {
        if (!s_hostTaskRunning) {
            ESP_LOGE(TAG, "Host sync timeout (host task not running)");
        } else {
            ESP_LOGE(TAG, "Host sync timeout (onSync not called)");
        }
        (void)stopBleOta();
        s_starting = false;
        return ESP_ERR_TIMEOUT;
    }
    if (!s_advertising) {
        ESP_LOGE(TAG, "Sync OK but advertising start failed");
        (void)stopBleOta();
        s_starting = false;
        return ESP_FAIL;
    }

    s_starting = false;
    ESP_LOGI(TAG, "=== BLE OTA init done (advertising) ===");
    return ESP_OK;
}

esp_err_t stopBleOta(void)
{
    if (!s_active && !s_starting) return ESP_OK;

    ESP_LOGI(TAG, "=== BLE OTA shutdown ===");

    if (ota::isInProgress()) {
        ota::abort();
    }

    // 1. 愿묎퀬/?곌껐 以묒?
    ble_gap_adv_stop();
    if (s_connHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_connHandle, BLE_ERR_REM_USER_CONN_TERM);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 2. NimBLE host 以묒? ??nimble_port_run() 由ы꽩
    int rc = nimble_port_stop();
    ESP_LOGI(TAG, "nimble_port_stop: %d", rc);
    if (rc == BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "nimble_port_stop: host already stopped");
        rc = 0;
    } else if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_stop unexpected rc=%d (continuing cleanup)", rc);
    }

    // 3. Wait until host task exits and deinit callback runs
    if (s_hostTaskRunning) {
        if (!waitForFalse(s_hostTaskRunning, HOST_TASK_STOP_TIMEOUT_TICKS)) {
            ESP_LOGW(TAG, "Host task stop timeout");
            if (s_hostTaskHandle != nullptr) {
                ESP_LOGW(TAG, "Force deleting stuck nimble_host task");
                vTaskDelete(s_hostTaskHandle);
                s_hostTaskHandle = nullptr;
                s_hostTaskRunning = false;
            }
        }
    }
    freeNimbleHostTaskBuffers();

    // 4. NimBLE ?ㅽ깮 ?댁젣
    esp_err_t ret = nimble_port_deinit();
    ESP_LOGI(TAG, "nimble_port_deinit: %s", esp_err_to_name(ret));
    if (ret == ESP_ERR_INVALID_STATE) {
        // Host already down; continue with explicit controller cleanup.
        ret = ESP_OK;
    }

    // 5. BT 而⑦듃濡ㅻ윭 ?곹깭 ?뺤씤 + 紐낆떆???뺣━
    forceBtControllerIdle();

    esp_bt_controller_status_t status = esp_bt_controller_get_status();
    if (status != ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_LOGE(TAG, "BT controller NOT idle (status=%d)!", status);
    }

    s_active = false;
    s_starting = false;
    s_advertising = false;
    s_synced = false;
    s_hostTaskHandle = nullptr;
    s_connHandle = BLE_HS_CONN_HANDLE_NONE;
    ESP_LOGI(TAG, "=== BLE OTA shutdown OK (heap: %lu) ===",
             (unsigned long)esp_get_free_heap_size());

    if (status != ESP_BT_CONTROLLER_STATUS_IDLE || ret != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool isBleOtaActive(void)
{
    return s_active;
}

bool isBleOtaAdvertising(void)
{
    return s_active && s_advertising;
}
