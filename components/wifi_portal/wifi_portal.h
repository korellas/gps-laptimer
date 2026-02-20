/**
 * @file wifi_portal.h
 * @brief WiFi Captive Portal with WebSocket log viewer and settings
 *
 * SoftAP "LAPTIMER" (WPA2) + DNS redirect + HTTP server + WebSocket log streaming
 * WiFi는 기본 OFF — 시리얼 'w' 또는 API로 켜고, 5분 무접속 시 자동 OFF
 */

#ifndef WIFI_PORTAL_H
#define WIFI_PORTAL_H

// SoftAP identity — used by both wifi_portal.cpp and the display layer
#define WIFI_AP_SSID     "LAPTIMER"
#define WIFI_AP_PASSWORD "laptimer"
#define WIFI_AP_IP_STR   "192.168.4.1"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SPIFFS에서 설정 로드 -> gApp.phoneNumber 등
 * init_storage() 이후, initDisplay() 이전에 호출
 */
void loadSettings(void);

/**
 * @brief 설정 저장 (SPIFFS JSON)
 */
void saveSettings(void);

/**
 * @brief WiFi 시스템 초기화 (1회). 라디오는 시작하지 않음.
 * loadSettings() 이후에 호출
 */
void initWifiPortal(void);

/**
 * @brief WiFi SoftAP + 서비스 시작
 */
void startWifiPortal(void);

/**
 * @brief WiFi SoftAP + 서비스 중지 (전력 절약)
 */
void stopWifiPortal(void);

/**
 * @brief WiFi 토글 (ON↔OFF)
 */
void toggleWifiPortal(void);

/**
 * @brief WiFi 활성 상태 확인
 */
bool isWifiPortalActive(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_PORTAL_H
