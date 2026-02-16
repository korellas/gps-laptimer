/**
 * @file sdcard_manager.h
 * @brief SD card manager (SDMMC 1-bit, FAT32)
 *
 * Hardware: GPIO39(CMD), GPIO40(D0), GPIO41(CLK)
 * Mount point: /sdcard
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 마운트/언마운트
bool sdcardInit(void);
void sdcardDeinit(void);

// 포맷 (마운트 해제 후 포맷 후 재마운트)
bool sdcardFormat(void);

// 상태
bool sdcardIsMounted(void);

// 용량 정보 (바이트)
uint64_t sdcardGetFreeBytes(void);
uint64_t sdcardGetTotalBytes(void);

#ifdef __cplusplus
}
#endif
