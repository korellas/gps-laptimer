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
#include "sd_logger.h"

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

// 세션 상태머신
static GPSSessionState s_sessionState = GPSSessionState::PRE_TRACK;

// 동적 섹터 경계 (활성 트랙에서 setupSectorBoundariesFromActiveTrack()으로 설정)
static SectorBoundaryPoint s_sectorBoundaries[MAX_SECTORS_PER_LAYOUT];
static int s_sectorBoundaryCount = 0;

// ============================================================
// HELPER: millis()
// ============================================================

static inline unsigned long gpsMillis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

// ============================================================
// HELPER: 활성 트랙에서 섹터 경계 설정
// ============================================================

static void setupSectorBoundariesFromActiveTrack() {
    s_sectorBoundaryCount = 0;

    const ActiveTrack& active = getActiveTrackConst();
    if (!active.isValid() || !active.layout->hasSectors()) {
        ESP_LOGI(TAG, "No active track/sectors for boundary setup");
        return;
    }

    int sectorCount = active.layout->sectorCount;
    // Sector boundaries are between sectors, so at most (sectorCount - 1) boundaries
    for (int i = 0; i < sectorCount - 1 && s_sectorBoundaryCount < MAX_SECTORS_PER_LAYOUT; i++) {
        const Sector* sec = active.layout->getSector(i);
        if (!sec) continue;

        if (sec->usesBoundaryLine()) {
            // Midpoint of boundary line as the representative point
            s_sectorBoundaries[s_sectorBoundaryCount].lat =
                (sec->boundaryLat1 + sec->boundaryLat2) / 2.0;
            s_sectorBoundaries[s_sectorBoundaryCount].lng =
                (sec->boundaryLng1 + sec->boundaryLng2) / 2.0;
            s_sectorBoundaryCount++;
        }
        // Distance-based sectors don't need explicit boundary points here
        // (handled by updateSectorDistancesFromReference)
    }

    ESP_LOGI(TAG, "Sector boundaries: %d points from track '%s'",
             s_sectorBoundaryCount, active.track->id);
}

// ============================================================
// HELPER: 세션 시작 (스타트라인 통과 시)
// ============================================================

static void startSession(unsigned long nowMs) {
    gpsState.lapStartMs = nowMs;
    gpsState.lapStarted = true;

    startRecordingLap(gApp.currentSessionNumber, gApp.currentLapNumber);

    // SD 카드 세션 CSV 시작
    sdSessionStart();
    sdLogEvent(nowMs, "SESSION", "START");

    // 동적 섹터 경계 설정
    setupSectorBoundariesFromActiveTrack();

    // 섹터 타이밍 초기화 (레퍼런스 거리 계산 전후 공통)
    resetSectorTiming();

    // 레퍼런스 랩이 있으면 섹터 경계 거리 계산
    if (gApp.hasValidReferenceLap && !gApp.referenceLap.points.empty()
            && s_sectorBoundaryCount > 0) {
        updateSectorDistancesFromReference(
            gApp.referenceLap.points.data(),
            gApp.referenceLap.cumulativeDistances.data(),
            (int)gApp.referenceLap.points.size(),
            s_sectorBoundaries, s_sectorBoundaryCount);
    }

    s_sessionState = GPSSessionState::SESSION_ACTIVE;

    // PRE_TRACK 디스플레이 해제
    setPreTrackMode(false, nullptr);

    ESP_LOGI(TAG, "Session started (lapStartMs=%lu)", nowMs);
}

// ============================================================
// PUBLIC API IMPLEMENTATION
// ============================================================

static void initializeGPSProcessor() {
    gpsState.reset();
    ESP_LOGI(TAG, "GPS Processor initialized");
}

void initializeGPSMode() {
    // GPS 모듈 활성화 (부팅 시 이미 ON — no-op)
    enableGPSModule();

    initializeGPSProcessor();

    // SD 카드 로거 초기화 (GPS 모드 진입 시)
    sdLoggerInit();
    sdLogEvent(0, "GPS_MODE", "INIT");

    // 세션 상태 초기화 (트랙 감지는 processRealGPS()의 PRE_TRACK 상태에서)
    s_sessionState = GPSSessionState::PRE_TRACK;
    s_sectorBoundaryCount = 0;
    resetTrackManager();

    // 섹터 타이밍 초기화
    initSectorTiming();

    // GPS 필터 초기화
    initGPSFilter();

    // Dead Reckoning 초기화
    initDeadReckoning();

    // 완료 표시 초기화
    lframe.lastCompletedLapMs = 0;
    lframe.lapCompleteDisplayEndMs = 0;

    // PRE_TRACK 디스플레이 진입
    setPreTrackMode(true, nullptr);

    ESP_LOGI(TAG, "GPS mode initialized → PRE_TRACK (ref=%s)",
             gApp.hasValidReferenceLap ? "YES" : "NO");
}

void resetRealGPS() {
    gpsState.lapStartMs = 0;
    gpsState.lapStarted = false;
    gpsState.lastValidGpsMs = 0;
    gpsState.gpsSignalLost = false;

    s_sessionState = GPSSessionState::PRE_TRACK;
    s_sectorBoundaryCount = 0;

    resetCrossingState();
    cancelRecording();
    resetSectorTiming();
    resetGPSFilter();
    resetDeadReckoning();
    resetTrackManager();

    // 세션 CSV 닫기
    sdSessionEnd();
    sdLogEvent(0, "GPS_MODE", "RESET");

    setPreTrackMode(true, nullptr);

    ESP_LOGI(TAG, "GPS mode reset → PRE_TRACK");
}

void processRealGPS() {
    unsigned long now = gpsMillis();

    // ─── GPS 신호 타임아웃 체크 (SESSION_ACTIVE에서만) ───
    if (s_sessionState == GPSSessionState::SESSION_ACTIVE &&
        gpsState.lapStarted && gpsState.lastValidGpsMs > 0) {
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

            if (isDeadReckoningActive()) stopDeadReckoning();

            // ─── GPSPoint 생성 ───
            GPSPoint point;
            point.set(ubx.lat, ubx.lon);
            point.speedKmh = ubx.speedKmh;
            point.headingDeg = ubx.headingDeg;
            point.gpsTimeMs = ubx.iTOW;

            // ─── GPS 필터 ───
            FilteredGPSPoint filtered = filterGPSPoint(point, now);
            if (filtered.wasSpikeFiltered || filtered.isValid) {
                point.lat = filtered.point.lat;
                point.lng = filtered.point.lng;
            }

            // ═══ 세션 상태머신 ═══
            switch (s_sessionState) {

            case GPSSessionState::PRE_TRACK: {
                // 트랙 근접 감지
                bool near = updateTrackProximity(point.lat, point.lng);
                if (near) {
                    const char* trackName = getNearTrackName();
                    ESP_LOGI(TAG, "PRE_TRACK → NEAR_TRACK (%s)",
                             trackName ? trackName : "?");

                    // 피니시라인 설정: SPIFFS 미설정 시 트랙 정의에서 로드
                    if (!isFinishLineConfigured()) {
                        const FinishLineDefinition* fl =
                            getActiveTrackConst().getFinishLineDefinition();
                        if (fl) setFinishLineFromDefinition(*fl);
                    }

                    s_sessionState = GPSSessionState::NEAR_TRACK;
                    setPreTrackMode(true, trackName);
                    sdLogEvent(now, "TRACK", trackName ? trackName : "?");
                }
                // SD 로그: PRE_TRACK GPS
                sdLogGPS(now, point.lat, point.lng, point.speedKmh, point.headingDeg,
                         ubx.fixType, ubx.satellites, "PRE_TRACK", 0);
                // PRE_TRACK: 디스플레이는 setPreTrackMode(true)가 처리
                gApp.previousPoint = gApp.currentPoint;
                gApp.currentPoint = point;
                gotNewData = true;
                break;
            }

            case GPSSessionState::NEAR_TRACK: {
                // 근접 이탈 체크 (히스테리시스)
                bool stillNear = updateTrackProximity(point.lat, point.lng);
                if (!stillNear) {
                    ESP_LOGI(TAG, "NEAR_TRACK → PRE_TRACK (left proximity)");
                    s_sessionState = GPSSessionState::PRE_TRACK;
                    setPreTrackMode(true, nullptr);
                    sdLogEvent(now, "TRACK", "LEFT_PROXIMITY");
                    gApp.previousPoint = gApp.currentPoint;
                    gApp.currentPoint = point;
                    gotNewData = true;
                    break;
                }

                // SESSION_START_MIN_SPEED_KMH 이상에서 스타트라인 통과 감지
                if (point.speedKmh >= SESSION_START_MIN_SPEED_KMH) {
                    if (checkFirstLineCrossing(point.lat, point.lng, point.headingDeg)) {
                        startSession(now);
                        // SESSION_ACTIVE로 전환됨 — lapStartMs는 startSession()이 설정
                        unsigned long lapTimeMs = 0;
                        point.lapTimeMs = lapTimeMs;
                        gApp.previousPoint = gApp.currentPoint;
                        gApp.currentPoint = point;
                        if (isRecording()) {
                            addPointToRecording(point.lat, point.lng, lapTimeMs,
                                                point.speedKmh, point.headingDeg);
                        }
                        gotNewData = true;
                        break;
                    }
                }

                // SD 로그: NEAR_TRACK GPS
                sdLogGPS(now, point.lat, point.lng, point.speedKmh, point.headingDeg,
                         ubx.fixType, ubx.satellites, "NEAR_TRACK", 0);
                // NEAR_TRACK: 속도+시각 표시 유지 (PRE_TRACK 디스플레이)
                gApp.previousPoint = gApp.currentPoint;
                gApp.currentPoint = point;
                gotNewData = true;
                break;
            }

            case GPSSessionState::SESSION_ACTIVE: {
                unsigned long lapTimeMs = now - gpsState.lapStartMs;
                point.lapTimeMs = lapTimeMs;

                // 포인트 기록
                if (isRecording()) {
                    addPointToRecording(point.lat, point.lng, lapTimeMs,
                                        point.speedKmh, point.headingDeg);
                }

                // SD 로그: SESSION_ACTIVE GPS + 세션 CSV
                sdLogGPS(now, point.lat, point.lng, point.speedKmh, point.headingDeg,
                         ubx.fixType, ubx.satellites, "SESSION_ACTIVE", lapTimeMs);
                sdSessionPoint(gApp.currentLapNumber, lapTimeMs,
                               point.lat, point.lng, point.speedKmh, point.headingDeg);

                // ─── 피니시라인 감지 ───
                if (lapTimeMs > MIN_LAP_TIME_MS) {
                    if (checkLineCrossing(point.lat, point.lng, point.headingDeg, lapTimeMs)) {
                        char lapMsg[32];
                        snprintf(lapMsg, sizeof(lapMsg), "LAP%u:%lums",
                                 (unsigned)gApp.currentLapNumber, lapTimeMs);
                        sdLogEvent(now, "LAP_COMPLETE", lapMsg);
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
                    gApp.currentDelta = calculateDelta(point, gApp.referenceLap,
                                                       &gApp.previousPoint);
                    // ─── 섹터 타이밍 ───
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
                break;
            }
            } // end switch
        }
    }

    // ─── Dead Reckoning (SESSION_ACTIVE에서만) ───
    if (s_sessionState == GPSSessionState::SESSION_ACTIVE) {
        if (!gotNewData && gpsState.lapStarted && gpsState.lastValidGpsMs > 0) {
            unsigned long elapsed = now - gpsState.lastValidGpsMs;

            if (elapsed > DEAD_RECKONING_ACTIVATION_DELAY_MS && !isDeadReckoningActive() &&
                gApp.currentPoint.speedKmh >= MIN_DEAD_RECKONING_SPEED_KMH) {
                startDeadReckoning(gApp.currentPoint,
                                  gApp.currentPoint.speedKmh,
                                  gApp.currentPoint.headingDeg, now);
            }

            if (isDeadReckoningActive() && !isDeadReckoningExpired()) {
                GPSPoint estimated = updateDeadReckoning(now);
                unsigned long lapTimeMs = now - gpsState.lapStartMs;
                estimated.lapTimeMs = lapTimeMs;
                updateDisplayData(estimated, gApp.currentDelta, lapTimeMs,
                                gApp.currentDelta.refTimeSec);
                return;
            }
        }

        // ─── 디스플레이 연속 업데이트 ───
        unsigned long lapTimeMs = gpsState.lapStarted ? (now - gpsState.lapStartMs) : 0;
        updateDisplayData(gApp.currentPoint, gApp.currentDelta,
                         lapTimeMs, gApp.currentDelta.refTimeSec);
    }
    // PRE_TRACK / NEAR_TRACK: updateLapData()가 setPreTrackMode로 처리
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
    return s_sectorBoundaryCount;
}

GPSSessionState getGPSSessionState() {
    return s_sessionState;
}

bool isGPSPreSession() {
    return s_sessionState != GPSSessionState::SESSION_ACTIVE;
}
