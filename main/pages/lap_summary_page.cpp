/**
 * @file lap_summary_page.cpp
 * @brief Lap summary page — shown after session ends (speed < 20 km/h).
 *
 * Displays session laps sorted by best time:
 *   LAP | TIME | S1 .. S6 | MIN | MAX
 *
 * Gestures:
 *   Swipe LEFT/RIGHT : scroll sector columns
 *   Swipe UP/DOWN    : scroll lap rows
 *   Swipe DOWN at top: exit to PRE_TRACK
 */

#include "page.h"
#include "page_manager.h"
#include "display_widgets.h"
#include "types.h"
#include "gps_processor.h"

struct LapSummaryPage : Page {
    int lapRowOffset    = 0;
    int sectorColOffset = 0;
    int maxSectors      = 0;

    LapSummaryPage()
        : Page(PageId::LAP_SUMMARY,
               /*gps=*/false, /*wifi=*/false, /*ble=*/false,
               ProcessingMode::NONE,
               /*sleep=*/true, SleepPolicy::IDLE_SLEEP)
    {}

    void onEnter(Page* from) override {
        lapRowOffset    = 0;
        sectorColOffset = 0;

        // 세션 전체에서 최대 섹터 수 계산
        maxSectors = 0;
        for (int i = 0; i < gApp.sessionLapCount; i++) {
            if (gApp.sessionLaps[i].sectorCount > maxSectors) {
                maxSectors = gApp.sessionLaps[i].sectorCount;
            }
        }

        applyPageVisibilityForPage(PageId::LAP_SUMMARY);
        updateLapSummaryDisplay(lapRowOffset, sectorColOffset);
    }

    void onUpdate() override {
        // 정적 화면 — 스와이프 이벤트 시에만 갱신되므로 여기선 아무것도 하지 않음
    }

    void onGesture(Gesture g) override {
        const int DATA_ROWS   = 6;                       // 표시 가능한 데이터 행 수
        const int SEC_VISIBLE = 6;                       // 한 번에 보이는 섹터 열 수

        switch (g) {
        case Gesture::SWIPE_LEFT:
            // 섹터 오른쪽으로 스크롤 (다음 섹터 그룹)
            if (maxSectors > SEC_VISIBLE && sectorColOffset + SEC_VISIBLE < maxSectors) {
                sectorColOffset++;
                updateLapSummaryDisplay(lapRowOffset, sectorColOffset);
            }
            break;

        case Gesture::SWIPE_RIGHT:
            // 섹터 왼쪽으로 스크롤
            if (sectorColOffset > 0) {
                sectorColOffset--;
                updateLapSummaryDisplay(lapRowOffset, sectorColOffset);
            }
            break;

        case Gesture::SWIPE_UP:
            // 랩 목록 아래로 스크롤 (다음 랩 그룹)
            if (lapRowOffset + DATA_ROWS < gApp.sessionLapCount) {
                lapRowOffset++;
                updateLapSummaryDisplay(lapRowOffset, sectorColOffset);
            }
            break;

        case Gesture::SWIPE_DOWN:
            if (lapRowOffset > 0) {
                // 랩 목록 위로 스크롤
                lapRowOffset--;
                updateLapSummaryDisplay(lapRowOffset, sectorColOffset);
            } else {
                // 최상단에서 아래로 스와이프 → PRE_TRACK으로 복귀
                // GPS 세션 상태도 PRE_TRACK으로 리셋
                resetRealGPS();
                gApp.sessionLapCount = 0;
                gPageManager.navigateTo(PageId::PRE_TRACK);
            }
            break;

        default:
            break;
        }
    }
};

static LapSummaryPage s_page;

Page* createLapSummaryPage() { return &s_page; }
