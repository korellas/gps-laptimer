#pragma once

#include <cstddef>
#include <cstdint>

namespace ota {

enum OtaErrorCode : uint8_t {
    OTA_ERR_NONE = 0,
    OTA_ERR_IMAGE_TOO_LARGE = 1,
    OTA_ERR_VALIDATION_FAILED = 2,
    OTA_ERR_LOW_BATTERY = 3,
    OTA_ERR_WRITE_FAILED = 4,
    OTA_ERR_NO_PARTITION = 5,
    OTA_ERR_BEGIN_FAILED = 6,
    OTA_ERR_SET_BOOT_FAILED = 7,
    OTA_ERR_INTERNAL_STACK = 8,
    OTA_ERR_BUSY = 9,
    OTA_ERR_TIMEOUT = 10,
    OTA_ERR_ABORTED = 11,
};

bool begin(uint32_t imageSize);
bool write(const uint8_t* data, size_t len);
bool end();
void abort();
void confirmBoot();
const char* getRunningVersion();
bool isInProgress();
uint8_t getLastErrorCode();
void setLastError(uint8_t code, const char* message);
void clearLastError();

} // namespace ota
