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
#include "ublox_gps.h"
#include "finish_line.h"
#include "lap_storage.h"
#include "waveshare_display.h"
#include "protocol.hpp"
#include "../timing/delta_calculator.h"
#include "../timing/sector_timing.h"
#include "../geo/gps_filter.h"
#include "../geo/dead_reckoning.h"
#include "../geo/geo_utils.h"
#include "../track/track_manager.h"
#include "sd_logger.h"
#include "sensor_fusion.h"

#include <cstdio>
#include <cmath>

#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "GPS_PROC";

// ============================================================
// EXTERNAL REFERENCES
// ============================================================

extern void updateDisplayData(const GPSPoint& point, const DeltaResult& delta,
                              unsigned long lapTimeMs, float refTimeSec);
extern void onLapComplete(unsigned long lapTimeMs);
extern bool loadReferenceLapFromStorage(void);

// ============================================================
// MODULE STATE
// ============================================================

static GPSProcessorState gpsState;

// 세션 상태머신
static GPSSessionState s_sessionState = GPSSessionState::PRE_TRACK;

// 동적 섹터 경계 (활성 트랙에서 setupSectorBoundariesFromActiveTrack()으로 설정)
static SectorBoundaryPoint s_sectorBoundaries[MAX_SECTORS_PER_LAYOUT];
static int s_sectorBoundaryCount = 0;

// Sensor fusion: 캘리브레이션 시도 타이머
static unsigned long s_lastCalibAttemptMs = 0;
static constexpr unsigned long CALIB_ATTEMPT_INTERVAL_MS = 10000;  // 10초

// PRE_TRACK: 트랙별 이전 GPS 포인트 (피니시라인 교차 감지용)
// setFinishLineFromDefinition()을 루프 내에서 호출하면 resetCrossingState()가 실행되어
// hasPrevPoint가 초기화되므로, 교차 감지 성공 전까지는 전역 상태를 건드리지 않음.
struct PreTrackPrev {
    double prevLat;
    double prevLng;
    bool hasPrevPoint;
};
static PreTrackPrev s_preTrackPrev[BUILTIN_TRACK_COUNT];

// 저속 세션 종료 타이머
static unsigned long s_lowSpeedStartMs = 0;
static constexpr float   SESSION_END_MAX_SPEED_KMH = 20.0f;
static constexpr unsigned long SESSION_END_DURATION_MS = 3000; // 3초 지속 시 종료

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

    startRecordingLap(gApp.currentSessionNumber, gApp.currentLapNumber,
                      getActiveTrackIndex(), getActiveTrackLayoutIndex());

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

    // Sensor fusion: 칼만 필터 초기화
    if (axisCalibIsValid()) {
        speedKfInit(gApp.currentPoint.speedKmh / 3.6f);
        gApp.fusionActive = true;
        gApp.fusionInDR = false;
        ESP_LOGI(TAG, "Sensor fusion active (speed KF initialized)");
    } else {
        ESP_LOGI(TAG, "Sensor fusion inactive (no axis calibration)");
    }

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
    s_lowSpeedStartMs = 0;
    for (auto& p : s_preTrackPrev) p = {};
    resetTrackManager();

    // 섹터 타이밍 초기화
    initSectorTiming();

    // GPS 필터 초기화
    initGPSFilter();

    // Dead Reckoning 초기화
    initDeadReckoning();

    // Sensor Fusion 초기화 (축 캘리브레이션 로드)
    axisCalibInit();
    if (gApp.imuCalibration.calibrated) {
        axisCalibSetGravity(gApp.imuCalibration.gravityX,
                            gApp.imuCalibration.gravityY,
                            gApp.imuCalibration.gravityZ);
    }
    if (axisCalibLoad()) {
        ESP_LOGI(TAG, "Sensor fusion: loaded calibration (residual=%.3f)",
                 axisCalibGetResidual());
    } else {
        ESP_LOGI(TAG, "Sensor fusion: no saved calibration, will collect during driving");
    }
    speedKfReset();
    gApp.fusionActive = false;
    gApp.fusionInDR = false;

    // 완료 표시 초기화
    lframe.lastCompletedLapMs = 0;
    lframe.lapCompleteDisplayEndMs = 0;

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
    s_lowSpeedStartMs = 0;
    for (auto& p : s_preTrackPrev) p = {};

    resetCrossingState();
    cancelRecording();
    resetSectorTiming();
    resetGPSFilter();
    resetDeadReckoning();
    resetTrackManager();

    // Sensor fusion: 샘플 버퍼 클리어 (저장된 캘리브레이션은 유지)
    axisCalibReset();
    speedKfReset();
    gApp.fusionActive = false;
    gApp.fusionInDR = false;

    // 세션 CSV 및 디버그 로그 닫기
    sdSessionEnd();
    sdLogEvent(0, "GPS_MODE", "RESET");
    sdLoggerClose();  // flush + close debug log so all GPS data reaches disk

    ESP_LOGI(TAG, "GPS mode reset → PRE_TRACK");
}

// ============================================================
// HELPER: 센서 퓨전 캘리브레이션 피드 + 주기적 solve
// ============================================================

static void feedCalibrationSample(const UBloxData& ubx, unsigned long now)
{
    if (!gApp.imuReady || !ubx.valid) return;

    float aSensor[3] = {
        gApp.imuData.accelX,
        gApp.imuData.accelY,
        gApp.imuData.accelZ
    };

    axisCalibFeedGPS(ubx.velNorthMps, ubx.velEastMps, ubx.velDownMps,
                     ubx.speedKmh, ubx.iTOW, aSensor);

    // 주기적으로 캘리브레이션 시도
    if ((now - s_lastCalibAttemptMs) >= CALIB_ATTEMPT_INTERVAL_MS) {
        s_lastCalibAttemptMs = now;
        if (axisCalibSolve()) {
            axisCalibSave();
            gApp.fusionCalibDoneMs = now;
        }
    }
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
                // 모든 트랙의 피니시라인을 체크하여 트랙 자동 식별 + 세션 시작.
                //
                // 핵심 제약: setFinishLineFromDefinition()은 내부에서 resetCrossingState()를
                // 호출해 hasPrevPoint를 초기화하므로, 루프 내 반복 호출 시 교차 감지가
                // 구조적으로 불가능함. 따라서:
                //  1) 트랙별 이전 포인트를 s_preTrackPrev[]로 독립 관리
                //  2) segmentsIntersect + 베어링 체크를 인라인으로 수행 (전역 상태 불변)
                //  3) setFinishLineFromDefinition()은 교차 감지 성공 후 1회만 호출
                //  4) prev 갱신은 체크 이후 (속도 조건 외부)에서 항상 수행 →
                //     속도 임계값 재진입 시 직전 GPS 포인트가 항상 유효하게 유지됨
                {
                    for (int ti = 0; ti < BUILTIN_TRACK_COUNT; ti++) {
                        const TrackDefinition* track = BUILTIN_TRACKS[ti];
                        PreTrackPrev& prev = s_preTrackPrev[ti];
                        if (!prev.hasPrevPoint) continue;  // 첫 포인트: 하단에서 설정

                        for (int li = 0; li < track->layoutCount; li++) {
                            const TrackLayout* layout = track->getLayout(li);
                            if (!layout || !layout->finishLine.isConfigured()) continue;
                            if (point.speedKmh < layout->minCrossSpeedKmh) continue;
                            const FinishLineDefinition& fl = layout->finishLine;

                            // 베어링 체크 (인라인 — 전역 crossing state 불변)
                            uint16_t h = (uint16_t)(((int)point.headingDeg % 360 + 360) % 360);
                            uint16_t bMin = (uint16_t)fl.validHeadingMin;
                            uint16_t bMax = (uint16_t)fl.validHeadingMax;
                            bool bearingOk = (bMin <= bMax) ? (h >= bMin && h <= bMax)
                                                             : (h >= bMin || h <= bMax);
                            if (!bearingOk) continue;

                            // 이전→현재 경로가 피니시라인과 교차하는지 검사
                            if (segmentsIntersect(prev.prevLat, prev.prevLng,
                                                  point.lat, point.lng,
                                                  fl.lat1, fl.lng1,
                                                  fl.lat2, fl.lng2)) {
                                // 트랙 식별 완료 — 이제 전역 finish line 1회 설정
                                setActiveTrack(track, li);
                                setFinishLineFromDefinition(fl);
                                ESP_LOGI(TAG, "PRE_TRACK → SESSION_ACTIVE (%s / %s)",
                                         track->name, layout->name);
                                sdLogEvent(now, "TRACK", track->name);

                                // 레퍼런스 랩은 세션 내 첫 랩 완료 후 자동 설정됨
                                // (스토리지에서 로드하지 않음 — 당일 세션 베스트만 사용)
                                gApp.hasValidReferenceLap = false;
                                gApp.bestLapTimeMs = UINT32_MAX;
                                gApp.sessionLapCount = 0;
                                ESP_LOGI(TAG, "Session start: ref lap reset (in-session best only)");

                                startSession(now);
                                unsigned long lapTimeMs = 0;
                                point.lapTimeMs = lapTimeMs;
                                gApp.previousPoint = gApp.currentPoint;
                                gApp.currentPoint = point;
                                if (isRecording()) {
                                    addPointToRecording(point.lat, point.lng, lapTimeMs,
                                                        point.speedKmh, point.headingDeg);
                                }
                                gotNewData = true;
                                goto exit_state_machine;
                            }
                        }
                    }
                }

                // 이전 포인트 갱신 — 속도 조건 외부: 속도가 낮아졌다 다시 높아져도 직전 포인트 유효
                for (int ti = 0; ti < BUILTIN_TRACK_COUNT; ti++) {
                    s_preTrackPrev[ti].prevLat = point.lat;
                    s_preTrackPrev[ti].prevLng = point.lng;
                    s_preTrackPrev[ti].hasPrevPoint = true;
                }

                // Sensor fusion: 캘리브레이션 샘플 수집
                feedCalibrationSample(ubx, now);

                // SD 로그: PRE_TRACK GPS
                sdLogGPS(now, point.lat, point.lng, point.speedKmh, point.headingDeg,
                         ubx.fixType, ubx.satellites, "PRE_TRACK", 0);
                gApp.previousPoint = gApp.currentPoint;
                gApp.currentPoint = point;
                gotNewData = true;
                break;
            }

            case GPSSessionState::SESSION_ACTIVE: {
                unsigned long lapTimeMs = now - gpsState.lapStartMs;
                point.lapTimeMs = lapTimeMs;

                // Sensor fusion: GPS 속도로 칼만 업데이트 + heading 갱신
                if (gApp.fusionActive) {
                    speedKfUpdate(ubx.speedKmh / 3.6f);
                    gApp.fusedSpeedKmh = speedKfGetSpeedKmh();
                    gApp.fusionInDR = false;
                    gApp.lastGpsHeadingRad = ubx.headingDeg * (3.14159265f / 180.0f);
                }

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
                        s_lowSpeedStartMs = 0;  // 랩 완료 시 저속 타이머 리셋
                        ESP_LOGI(TAG, "New lap started");
                    }
                }

                // ─── 저속 세션 종료 감지 ───
                // 완료된 랩이 1개 이상이고, 속도가 20km/h 미만으로 3초 지속 시 서머리 표시
                if (gApp.sessionLapCount > 0) {
                    if (point.speedKmh < SESSION_END_MAX_SPEED_KMH) {
                        if (s_lowSpeedStartMs == 0) {
                            s_lowSpeedStartMs = now;
                        } else if ((now - s_lowSpeedStartMs) >= SESSION_END_DURATION_MS) {
                            ESP_LOGI(TAG, "SESSION_ENDING: speed=%.1f km/h for %lums → summary",
                                     point.speedKmh, (unsigned long)(now - s_lowSpeedStartMs));
                            cancelRecording();
                            sdSessionEnd();
                            sdLogEvent(now, "SESSION", "END_LOWSPEED");
                            s_sessionState = GPSSessionState::SESSION_ENDING;
                            s_lowSpeedStartMs = 0;
                        }
                    } else {
                        s_lowSpeedStartMs = 0;
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

            case GPSSessionState::SESSION_ENDING:
                // 페이지 전환 대기 중 — 처리 없음
                break;

            } // end switch
            exit_state_machine:;
        }
    }

    // ─── Dead Reckoning (SESSION_ACTIVE에서만) ───
    if (s_sessionState == GPSSessionState::SESSION_ACTIVE) {
        if (!gotNewData && gpsState.lapStarted && gpsState.lastValidGpsMs > 0) {
            unsigned long elapsed = now - gpsState.lastValidGpsMs;

            if (elapsed > DEAD_RECKONING_ACTIVATION_DELAY_MS) {
                // IMU-aided dead reckoning: 칼만 필터가 imuTask에서 계속 predict 중
                if (gApp.fusionActive && axisCalibIsValid()) {
                    if (!gApp.fusionInDR) {
                        gApp.fusionInDR = true;
                        ESP_LOGI(TAG, "IMU-aided dead reckoning started (speed=%.1f km/h)",
                                 gApp.fusedSpeedKmh);
                    }
                    // 칼만 필터 속도를 현재 포인트에 반영
                    gApp.currentPoint.speedKmh = gApp.fusedSpeedKmh;
                }

                // 위치 추정: 기존 constant-velocity DR 유지 (속도만 퓨전 값 사용)
                if (!isDeadReckoningActive() &&
                    gApp.currentPoint.speedKmh >= MIN_DEAD_RECKONING_SPEED_KMH) {
                    startDeadReckoning(gApp.currentPoint,
                                      gApp.currentPoint.speedKmh,
                                      gApp.currentPoint.headingDeg, now);
                }
            }

            if (isDeadReckoningActive() && !isDeadReckoningExpired()) {
                GPSPoint estimated = updateDeadReckoning(now);
                // IMU 퓨전 속도가 있으면 DR 추정 포인트에도 반영
                if (gApp.fusionActive && gApp.fusionInDR) {
                    estimated.speedKmh = gApp.fusedSpeedKmh;
                }
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
    // PRE_TRACK: PreTrackPage handles display via updatePreTrackDisplay()
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
    return s_sessionState == GPSSessionState::PRE_TRACK;
}
