#include "ota_manager.h"
#include "types.h"

#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_app_desc.h"

#include <cstring>

static const char* TAG = "OTA";

extern AppContext gApp;

// OTA 내부 상태
static esp_ota_handle_t s_otaHandle = 0;
static const esp_partition_t* s_updatePartition = nullptr;
static bool s_inProgress = false;

namespace ota {

bool begin(uint32_t imageSize) {
    if (s_inProgress) {
        ESP_LOGW(TAG, "OTA already in progress");
        return false;
    }

    // 배터리 체크
    if (gApp.batteryPercent >= 0 && gApp.batteryPercent < 30.0f) {
        ESP_LOGE(TAG, "Battery too low: %.0f%% (min 30%%)", gApp.batteryPercent);
        snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "Battery too low (%.0f%%)", gApp.batteryPercent);
        gApp.otaState = OTAState::ERROR;
        return false;
    }

    // 비활성 OTA 파티션 찾기
    s_updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (s_updatePartition == nullptr) {
        ESP_LOGE(TAG, "No OTA partition found");
        snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "No OTA partition");
        gApp.otaState = OTAState::ERROR;
        return false;
    }

    // 파티션 크기 체크
    if (imageSize > s_updatePartition->size) {
        ESP_LOGE(TAG, "Image too large: %lu > %lu", (unsigned long)imageSize, (unsigned long)s_updatePartition->size);
        snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "Image too large");
        gApp.otaState = OTAState::ERROR;
        return false;
    }

    ESP_LOGI(TAG, "Starting OTA: partition=%s, size=%lu", s_updatePartition->label, (unsigned long)imageSize);

    esp_err_t err = esp_ota_begin(s_updatePartition, imageSize, &s_otaHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "OTA begin: %s", esp_err_to_name(err));
        gApp.otaState = OTAState::ERROR;
        return false;
    }

    s_inProgress = true;
    gApp.otaState = OTAState::RECEIVING;
    gApp.otaTotalBytes = imageSize;
    gApp.otaReceivedBytes = 0;
    gApp.otaProgress = 0.0f;
    gApp.otaErrorMsg[0] = '\0';

    return true;
}

bool write(const uint8_t* data, size_t len) {
    if (!s_inProgress) {
        return false;
    }

    esp_err_t err = esp_ota_write(s_otaHandle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "Write error: %s", esp_err_to_name(err));
        gApp.otaState = OTAState::ERROR;
        abort();
        return false;
    }

    gApp.otaReceivedBytes += len;
    if (gApp.otaTotalBytes > 0) {
        gApp.otaProgress = (float)gApp.otaReceivedBytes / (float)gApp.otaTotalBytes;
    }

    return true;
}

bool end() {
    if (!s_inProgress) {
        return false;
    }

    gApp.otaState = OTAState::VALIDATING;

    esp_err_t err = esp_ota_end(s_otaHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (validation): %s", esp_err_to_name(err));
        snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "Validation: %s", esp_err_to_name(err));
        gApp.otaState = OTAState::ERROR;
        s_inProgress = false;
        return false;
    }

    err = esp_ota_set_boot_partition(s_updatePartition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        snprintf(gApp.otaErrorMsg, sizeof(gApp.otaErrorMsg), "Set boot: %s", esp_err_to_name(err));
        gApp.otaState = OTAState::ERROR;
        s_inProgress = false;
        return false;
    }

    ESP_LOGI(TAG, "OTA complete, new partition: %s", s_updatePartition->label);
    gApp.otaState = OTAState::COMPLETE;
    gApp.otaProgress = 1.0f;
    s_inProgress = false;

    return true;
}

void abort() {
    if (s_inProgress) {
        esp_ota_abort(s_otaHandle);
        s_inProgress = false;
        ESP_LOGW(TAG, "OTA aborted");
    }
    gApp.otaState = OTAState::IDLE;
    gApp.otaProgress = 0.0f;
    gApp.otaReceivedBytes = 0;
    gApp.otaTotalBytes = 0;
}

void confirmBoot() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Confirming OTA boot (cancel rollback)");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

const char* getRunningVersion() {
    const esp_app_desc_t* desc = esp_app_get_description();
    return desc->version;
}

bool isInProgress() {
    return s_inProgress;
}

} // namespace ota
