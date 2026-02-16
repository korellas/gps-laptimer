/**
 * @file gps_processor.cpp
 * @brief GPS hardware mode implementation for GPS Lap Timer
 * @version 2.0
 *
 * Handles real GPS data from u-blox module, lap recording,
 * finish line detection, sector timing, GPS filtering.
 */

#include "gps_processor.h"
#include "config.h"
#include "types.h"
#include "../ublox_gps.h"
#include "../finish_line.h"
#include "../lap_storage.h"
#include "../waveshare_display.h"
#include "../protocol.hpp"
#include "../timing/delta_calculator.h"
#include "../timing/sector_timing.h"
#include "../geo/gps_filter.h"
#include "../geo/dead_reckoning.h"
#include "../track/track_manager.h"

#include <cstdio>

#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "GPS_PROC";

// ============================================================
// EXTERNAL REFERENCES
// ============================================================

extern void updateDisplayData(const GPSPoint& point, const DeltaResult& delta,
                              unsigned long lapTimeMs, float refTimeSec);
extern void onLapComplete(unsigned long lapTimeMs);

// ============================================================
// MODULE STATE
// ============================================================

static GPSProcessorState gpsState;

// 에버랜드 섹터 경계 (SSOT: builtin_tracks.h → everland::SECTOR_BOUNDARIES)
static const SectorBoundaryPoint s_sectorBoundaries[] = {
    {37.2974004, 127.2175125},   // S1→S2
    {37.2961782, 127.2138076},   // S2→S3
};
static constexpr int SECTOR_BOUNDARY_COUNT = sizeof(s_sectorBoundaries) / sizeof(s_sectorBoundaries[0]);

// ============================================================
// HELPER: millis()
// ============================================================

static inline unsigned long gpsMillis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

// ============================================================
// PUBLIC API IMPLEMENTATION
// ============================================================

static void initializeGPSProcessor() {
    gpsState.reset();
    ESP_LOGI(TAG, "GPS Processor initialized");
}

void initializeGPSMode() {
    // GPS 모듈 활성화 (이미 활성화된 경우 no-op)
    enableGPSModule();

    initializeGPSProcessor();

    // 트랙 활성화 (시뮬레이션과 동일)
    if (setActiveTrackById("everland", "full")) {
        ESP_LOGI(TAG, "Track activated: everland/full");
    } else {
        ESP_LOGW(TAG, "Track activation failed");
    }

    // 섹터 타이밍 초기화
    initSectorTiming();

    // GPS 필터 초기화
    initGPSFilter();

    // Dead Reckoning 초기화
    initDeadReckoning();

    // 레퍼런스 랩이 있으면 섹터 경계 거리 계산
    if (gApp.hasValidReferenceLap && !gApp.referenceLap.points.empty()) {
        updateSectorDistancesFromReference(
            gApp.referenceLap.points.data(),
            gApp.referenceLap.cumulativeDistances.data(),
            (int)gApp.referenceLap.points.size(),
            s_sectorBoundaries, SECTOR_BOUNDARY_COUNT);
        resetSectorTiming();
        ESP_LOGI(TAG, "Sector distances calculated from reference (%d pts)",
                 (int)gApp.referenceLap.points.size());
    }

    // 완료 표시 초기화
    lframe.lastCompletedLapMs = 0;
    lframe.lapCompleteDisplayEndMs = 0;

    ESP_LOGI(TAG, "GPS mode initialized (ref=%s)",
             gApp.hasValidReferenceLap ? "YES" : "NO");
}

void resetRealGPS() {
    gpsState.lapStartMs = 0;
    gpsState.lapStarted = false;
    gpsState.lastValidGpsMs = 0;
    gpsState.gpsSignalLost = false;

    resetCrossingState();
    cancelRecording();
    resetSectorTiming();
    resetGPSFilter();
    resetDeadReckoning();

    ESP_LOGI(TAG, "GPS mode reset");
}

void processRealGPS() {
    unsigned long now = gpsMillis();

    // ─── GPS 신호 타임아웃 체크 ───
    if (gpsState.lapStarted && gpsState.lastValidGpsMs > 0) {
        if ((now - gpsState.lastValidGpsMs) > GPS_TIMEOUT_MS) {
            if (!gpsState.gpsSignalLost) {
                gpsState.gpsSignalLost = true;
                setGpsSignalLost(true);
                ESP_LOGW(TAG, "GPS signal lost");
            }
        }
    }

    bool gotNewData = false;

    // ─── UBX 데이터 업데이트 ───
    if (updateUBloxGPS()) {
        UBloxData ubx = getUBloxData();

        if (ubx.valid) {
            gpsState.lastValidGpsMs = now;

            if (gpsState.gpsSignalLost) {
                gpsState.gpsSignalLost = false;
                setGpsSignalLost(false);
                ESP_LOGI(TAG, "GPS signal recovered");
            }

            // Dead Reckoning 활성 중이면 중단
            if (isDeadReckoningActive()) {
                stopDeadReckoning();
            }

            // ─── GPSPoint 생성 (Phase 4: isValid 수정) ───
            GPSPoint point;
            point.set(ubx.lat, ubx.lon);  // initialized = true
            point.speedKmh = ubx.speedKmh;
            point.headingDeg = ubx.headingDeg;
            point.gpsTimeMs = ubx.iTOW;

            // 첫 유효 데이터에서 랩 시작
            if (!gpsState.lapStarted) {
                gpsState.lapStartMs = now;
                gpsState.lapStarted = true;
                startRecordingLap(gApp.currentSessionNumber, gApp.currentLapNumber);
                ESP_LOGI(TAG, "Lap started (sat=%d)", ubx.satellites);
            }

            unsigned long lapTimeMs = now - gpsState.lapStartMs;
            point.lapTimeMs = lapTimeMs;

            // ─── GPS 필터 (Phase 8) ───
            FilteredGPSPoint filtered = filterGPSPoint(point, now);
            if (filtered.wasSpikeFiltered) {
                // 스파이크 감지 — 필터된 좌표 사용 (마지막 유효 위치)
                point.lat = filtered.point.lat;
                point.lng = filtered.point.lng;
                // 속도/heading은 원본 유지
            } else if (filtered.isValid) {
                // 정상 — 스무딩된 좌표 사용
                point.lat = filtered.point.lat;
                point.lng = filtered.point.lng;
            }

            // 포인트 기록
            if (isRecording()) {
                addPointToRecording(point.lat, point.lng, lapTimeMs,
                                    point.speedKmh, point.headingDeg);
            }

            // ─── 피니시라인 감지 ───
            if (lapTimeMs > MIN_LAP_TIME_MS) {
                if (checkLineCrossing(point.lat, point.lng, point.headingDeg, lapTimeMs)) {
                    onLapComplete(lapTimeMs);
                    gpsState.lapStartMs = now;
                    ESP_LOGI(TAG, "New lap started");
                }
            }

            // 상태 업데이트
            gApp.previousPoint = gApp.currentPoint;
            gApp.currentPoint = point;

            // ─── 델타 계산 ───
            if (gApp.hasValidReferenceLap) {
                gApp.currentDelta = calculateDelta(point, gApp.referenceLap, &gApp.previousPoint);

                // ─── 섹터 타이밍 (Phase 5) ───
                if (gApp.currentDelta.trackDistanceM >= 0) {
                    int completedSector = checkSectorTransitionByDistance(
                        gApp.currentDelta.trackDistanceM);
                    if (completedSector >= 0) {
                        onSectorComplete(completedSector, lapTimeMs,
                                        gApp.currentDelta.deltaSeconds);
                        const CurrentSectorTiming& st = getCurrentSectorTiming();
                        int nextSector = completedSector + 1;
                        if (nextSector < st.totalSectors) {
                            onSectorEntry(nextSector, lapTimeMs);
                        }
                    }
                }
            } else {
                gApp.currentDelta.clear();
            }

            gotNewData = true;
        }
    }

    // ─── Dead Reckoning (Phase 9) ───
    if (!gotNewData && gpsState.lapStarted && gpsState.lastValidGpsMs > 0) {
        unsigned long elapsed = now - gpsState.lastValidGpsMs;

        // GPS가 200ms 이상 두절 시 DR 시작
        if (elapsed > 200 && !isDeadReckoningActive() &&
            gApp.currentPoint.speedKmh >= MIN_DEAD_RECKONING_SPEED_KMH) {
            startDeadReckoning(gApp.currentPoint,
                              gApp.currentPoint.speedKmh,
                              gApp.currentPoint.headingDeg, now);
        }

        // DR 활성 중이면 추정 위치로 디스플레이 업데이트
        if (isDeadReckoningActive() && !isDeadReckoningExpired()) {
            GPSPoint estimated = updateDeadReckoning(now);
            unsigned long lapTimeMs = now - gpsState.lapStartMs;
            estimated.lapTimeMs = lapTimeMs;
            updateDisplayData(estimated, gApp.currentDelta, lapTimeMs,
                            gApp.currentDelta.refTimeSec);
            return;  // DR 중에는 아래 일반 업데이트 스킵
        }
    }

    // ─── 디스플레이 연속 업데이트 (Phase 6) ───
    // GPS fix 여부와 무관하게 디스플레이 갱신 (fix 없어도 화면 멈춤 방지)
    {
        unsigned long lapTimeMs = gpsState.lapStarted ? (now - gpsState.lapStartMs) : 0;
        updateDisplayData(gApp.currentPoint, gApp.currentDelta,
                         lapTimeMs, gApp.currentDelta.refTimeSec);
    }
}

// ============================================================
// QUERY FUNCTIONS
// ============================================================

bool isGPSLapStarted() {
    return gpsState.lapStarted;
}

unsigned long getCurrentLapTimeMs() {
    if (!gpsState.lapStarted) {
        return 0;
    }
    return gpsMillis() - gpsState.lapStartMs;
}

unsigned long getLastValidGpsTimeMs() {
    return gpsState.lastValidGpsMs;
}

const GPSProcessorState& getGPSProcessorState() {
    return gpsState;
}

GPSProcessorState& getGPSProcessorStateMutable() {
    return gpsState;
}

// ============================================================
// 섹터 경계 접근 (onLapComplete에서 사용)
// ============================================================

const SectorBoundaryPoint* getGPSSectorBoundaries() {
    return s_sectorBoundaries;
}

int getGPSSectorBoundaryCount() {
    return SECTOR_BOUNDARY_COUNT;
}
