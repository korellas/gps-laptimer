/**
 * @file log_ringbuffer.h
 * @brief Lock-free ring buffer for capturing ESP_LOG output
 *
 * Single-producer (log hook) / single-consumer (WS flush timer) pattern.
 * Fixed 4KB buffer with overwrite-oldest semantics.
 */

#ifndef LOG_RINGBUFFER_H
#define LOG_RINGBUFFER_H

#include <cstring>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class LogRingBuffer {
public:
    static constexpr size_t BUF_SIZE = 4096;

    void init() {
        mutex_ = xSemaphoreCreateMutex();
        head_ = 0;
        tail_ = 0;
        full_ = false;
    }

    // 프로듀서: 논블로킹 쓰기 (mutex 획득 실패 시 드롭)
    void tryWrite(const char *data, size_t len) {
        if (!mutex_ || len == 0) return;
        if (xSemaphoreTake(mutex_, 0) != pdTRUE) return;  // 즉시 실패 → 드롭

        for (size_t i = 0; i < len; i++) {
            buf_[head_] = data[i];
            head_ = (head_ + 1) % BUF_SIZE;
            if (full_) {
                tail_ = (tail_ + 1) % BUF_SIZE;
            }
            full_ = (head_ == tail_);
        }

        xSemaphoreGive(mutex_);
    }

    // 프로듀서: 로그 메시지 기록 (블로킹)
    void write(const char *data, size_t len) {
        if (!mutex_ || len == 0) return;
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(5)) != pdTRUE) return;

        for (size_t i = 0; i < len; i++) {
            buf_[head_] = data[i];
            head_ = (head_ + 1) % BUF_SIZE;
            if (full_) {
                tail_ = (tail_ + 1) % BUF_SIZE;  // 가장 오래된 데이터 덮어쓰기
            }
            full_ = (head_ == tail_);
        }

        xSemaphoreGive(mutex_);
    }

    // 컨슈머: 버퍼 내용 읽기 (읽은 후 클리어)
    // dest에 복사, 반환값 = 복사된 바이트 수
    size_t read(char *dest, size_t maxLen) {
        if (!mutex_) return 0;
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) != pdTRUE) return 0;

        size_t available = full_ ? BUF_SIZE : ((head_ >= tail_) ? (head_ - tail_) : (BUF_SIZE - tail_ + head_));
        size_t toRead = (available < maxLen) ? available : maxLen;

        for (size_t i = 0; i < toRead; i++) {
            dest[i] = buf_[tail_];
            tail_ = (tail_ + 1) % BUF_SIZE;
        }
        full_ = false;

        xSemaphoreGive(mutex_);
        return toRead;
    }

    bool hasData() const {
        return full_ || (head_ != tail_);
    }

private:
    char buf_[BUF_SIZE] = {};
    size_t head_ = 0;
    size_t tail_ = 0;
    bool full_ = false;
    SemaphoreHandle_t mutex_ = nullptr;
};

#endif // LOG_RINGBUFFER_H
