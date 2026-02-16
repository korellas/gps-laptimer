#pragma once

#include <cstdint>
#include <cstddef>

namespace ota {

/**
 * @brief OTA 업데이트 시작
 * @param imageSize 전체 이미지 크기 (bytes)
 * @return true 성공, false OTA 파티션 열기 실패
 */
bool begin(uint32_t imageSize);

/**
 * @brief 펌웨어 청크 기록
 * @param data 데이터 포인터
 * @param len 데이터 길이
 * @return true 성공, false 쓰기 오류
 */
bool write(const uint8_t* data, size_t len);

/**
 * @brief OTA 완료 — 검증 + 부트 파티션 전환
 * @return true 성공 (리부트 필요), false 검증 실패
 */
bool end();

/**
 * @brief OTA 취소 — 진행 중인 OTA 중단
 */
void abort();

/**
 * @brief 부팅 시 호출 — 롤백 취소 (정상 부팅 확인)
 *
 * app_main() 초기화 완료 후 호출해야 함.
 * 롤백 펜딩 상태가 아니면 아무 동작 안 함.
 */
void confirmBoot();

/**
 * @brief 현재 실행 중인 앱 버전 반환
 */
const char* getRunningVersion();

/**
 * @brief OTA 진행 중 여부
 */
bool isInProgress();

} // namespace ota
