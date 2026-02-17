/**
 * @file ble_ota.h
 * @brief BLE OTA firmware update service (NimBLE)
 *
 * On-demand BLE OTA: startBleOta()로 NimBLE 시작 + 광고,
 * stopBleOta()로 종료. WiFi와 동시 사용 불가 (라디오 배타성).
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BLE OTA 모드 시작
 * NimBLE 초기화 + GATT 서비스 등록 + "LAPTIMER-OTA" 광고 시작.
 * WiFi 정지는 호출자 책임 (메모리 확보를 위해 esp_wifi_deinit 필요).
 * @return ESP_OK 성공, ESP_ERR_NO_MEM 메모리 부족, 기타 에러
 */
esp_err_t startBleOta(void);

/**
 * @brief BLE OTA 모드 중지
 * 진행 중인 OTA는 abort. NimBLE 스택 + BT 컨트롤러 완전 해제.
 * @return ESP_OK 성공, ESP_FAIL 정리 실패 (s_active는 미해제)
 */
esp_err_t stopBleOta(void);

/**
 * @brief BLE OTA 스택 활성 여부 (init 완료)
 */
bool isBleOtaActive(void);

/**
 * @brief BLE OTA 실제 광고 중인지 (onSync 이후 ble_gap_adv_start 성공)
 */
bool isBleOtaAdvertising(void);

#ifdef __cplusplus
}
#endif
