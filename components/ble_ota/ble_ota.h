/**
 * @file ble_ota.h
 * @brief BLE OTA firmware update service (NimBLE)
 *
 * On-demand BLE OTA: startBleOta()로 NimBLE 시작 + 광고,
 * stopBleOta()로 종료. WiFi와 동시 사용 불가 (라디오 배타성).
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BLE OTA 모드 시작
 * NimBLE 초기화 + GATT 서비스 등록 + "LAPTIMER-OTA" 광고 시작.
 * WiFi가 활성 상태면 자동으로 중지됨.
 */
void startBleOta(void);

/**
 * @brief BLE OTA 모드 중지
 * 진행 중인 OTA는 abort. NimBLE 스택 해제.
 */
void stopBleOta(void);

/**
 * @brief BLE OTA 활성 여부
 */
bool isBleOtaActive(void);

#ifdef __cplusplus
}
#endif
