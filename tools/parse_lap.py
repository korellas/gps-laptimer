#!/usr/bin/env python3
"""
parse_lap.py — GPS Laptimer 베스트랩 bin 파일 파서

사용법:
  python tools/parse_lap.py <bin파일경로>
  python tools/parse_lap.py best.bin
  python tools/parse_lap.py best.bin --sector-check  # 인제 섹터 경계 투영 분석
"""

import struct
import sys
import math
import argparse

# ─── 바이너리 포맷 ────────────────────────────────────────────
# LapHeader (32 bytes, pack=1)
HEADER_FMT   = "<IHHIIHHHHxxxxxxxx"  # 4+2+2+4+4+2+2+2+2+8 = 32 bytes
HEADER_SIZE  = struct.calcsize(HEADER_FMT)  # 32
LAP_MAGIC    = 0x4C415001

# StoredPoint (15 bytes, pack=1)
POINT_FMT    = "<iiIHB"  # int32 lat, int32 lng, uint32 lapTimeMs, uint16 speedX10, uint8 heading
POINT_SIZE   = struct.calcsize(POINT_FMT)  # 15

# ─── 인제스피디움 섹터 경계 좌표 ─────────────────────────────
INJE_SECTORS = [
    ("S1/S2", 38.004186, 128.293964),
    ("S2/S3", 37.998797, 128.291942),
    ("S3/S4", 37.999900, 128.285925),
]
INJE_FINISH  = (38.000583, 128.291111)

# ─── 거리 계산 ───────────────────────────────────────────────
def haversine(lat1, lng1, lat2, lng2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlng = math.radians(lng2 - lng1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlng/2)**2
    return R * 2 * math.asin(math.sqrt(a))

def flat_dist(lat1, lng1, lat2, lng2):
    """빠른 평면 거리 (short distances)"""
    cos_lat = math.cos(math.radians((lat1 + lat2) / 2))
    dlat_m = (lat2 - lat1) * 111320.0
    dlng_m = (lng2 - lng1) * 111320.0 * cos_lat
    return math.sqrt(dlat_m**2 + dlng_m**2)

def proj_t(px, py, ax, ay, bx, by):
    """점 P를 선분 AB에 투영한 t 파라미터 (0~1)"""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    denom = abx*abx + aby*aby
    if denom < 1e-12:
        return 0.0
    t = (apx*abx + apy*aby) / denom
    return max(0.0, min(1.0, t))

# ─── 파서 ────────────────────────────────────────────────────
def parse_bin(path):
    with open(path, "rb") as f:
        data = f.read()

    if len(data) < HEADER_SIZE:
        print(f"ERROR: 파일 너무 작음 ({len(data)} bytes)")
        sys.exit(1)

    hdr = struct.unpack_from(HEADER_FMT, data, 0)
    magic, version, point_count, total_ms, timestamp, max_spd_x10, avg_spd_x10, session_id, lap_id = hdr

    if magic != LAP_MAGIC:
        print(f"ERROR: 매직 불일치 0x{magic:08X} (expected 0x{LAP_MAGIC:08X})")
        sys.exit(1)

    print(f"=== LAP HEADER ===")
    print(f"  version     : {version}")
    print(f"  session/lap : {session_id}/{lap_id}")
    print(f"  points      : {point_count}")
    print(f"  totalTime   : {total_ms/1000:.3f}s  ({total_ms}ms)")
    print(f"  maxSpeed    : {max_spd_x10/10:.1f} km/h")
    print(f"  avgSpeed    : {avg_spd_x10/10:.1f} km/h")
    print(f"  fileSize    : {len(data)} bytes  (expected {HEADER_SIZE + point_count * POINT_SIZE})")
    print()

    points = []
    offset = HEADER_SIZE
    for i in range(point_count):
        if offset + POINT_SIZE > len(data):
            print(f"WARNING: 포인트 {i}에서 데이터 잘림")
            break
        lat_raw, lng_raw, lap_ms, spd_x10, hdg_raw = struct.unpack_from(POINT_FMT, data, offset)
        offset += POINT_SIZE
        lat     = lat_raw / 1e7
        lng     = lng_raw / 1e7
        spd_kmh = spd_x10 / 10.0
        hdg_deg = hdg_raw * (360.0 / 255.0)
        points.append((lat, lng, lap_ms, spd_kmh, hdg_deg))

    return points, total_ms

def analyze(points, total_ms, do_sector_check=False):
    print(f"=== 포인트 통계 ===")
    print(f"  총 포인트 : {len(points)}")

    if not points:
        return

    # 좌표 범위
    lats = [p[0] for p in points]
    lngs = [p[1] for p in points]
    spds = [p[3] for p in points]
    print(f"  lat 범위  : {min(lats):.6f} ~ {max(lats):.6f}")
    print(f"  lng 범위  : {min(lngs):.6f} ~ {max(lngs):.6f}")
    print(f"  속도 범위 : {min(spds):.1f} ~ {max(spds):.1f} km/h")

    # 누적 거리 계산
    cum_dist = [0.0]
    for i in range(1, len(points)):
        d = flat_dist(points[i-1][0], points[i-1][1], points[i][0], points[i][1])
        cum_dist.append(cum_dist[-1] + d)
    print(f"  트랙 총 거리: {cum_dist[-1]:.1f} m")

    # GPS 포인트 간격 분석
    gaps_ms = [points[i][2] - points[i-1][2] for i in range(1, len(points))]
    if gaps_ms:
        avg_gap = sum(gaps_ms) / len(gaps_ms)
        max_gap = max(gaps_ms)
        print(f"  평균 GPS 간격: {avg_gap:.0f}ms  최대: {max_gap}ms")

    # GPS 점프 이상값 탐지
    print()
    print("=== GPS 점프 이상값 (>30m) ===")
    found_jump = False
    for i in range(1, len(points)):
        d = flat_dist(points[i-1][0], points[i-1][1], points[i][0], points[i][1])
        dt_ms = points[i][2] - points[i-1][2]
        if d > 30:
            spd_inferred = (d / (dt_ms/1000.0)) * 3.6 if dt_ms > 0 else 999
            print(f"  pt{i:4d}: {d:.1f}m jump, dt={dt_ms}ms, inferred={spd_inferred:.0f}km/h  "
                  f"({points[i-1][0]:.6f},{points[i-1][1]:.6f}) → ({points[i][0]:.6f},{points[i][1]:.6f})")
            found_jump = True
    if not found_jump:
        print("  없음 (깨끗)")

    if do_sector_check:
        print()
        print("=== 인제스피디움 섹터 경계 근접 포인트 ===")
        for name, slat, slng in INJE_SECTORS:
            # 경계점 가장 가까운 10개 포인트
            dists = [(i, flat_dist(p[0], p[1], slat, slng)) for i, p in enumerate(points)]
            dists.sort(key=lambda x: x[1])
            top5 = dists[:5]
            print(f"\n  [{name}] 기준좌표=({slat:.6f},{slng:.6f})")
            for idx, d in top5:
                p = points[idx]
                pct = cum_dist[idx] / cum_dist[-1] * 100
                print(f"    pt{idx:4d}: dist={d:.1f}m  t={p[2]/1000:.2f}s  spd={p[3]:.1f}km/h  "
                      f"trackDist={cum_dist[idx]:.1f}m ({pct:.1f}%)")

        print()
        print("=== S4 구간 포인트 (마지막 섹터 경계 통과 후) ===")
        # S3/S4 경계에서 가장 가까운 포인트 찾기
        s3s4_lat, s3s4_lng = 37.999900, 128.285925
        s3s4_dists = [(i, flat_dist(p[0], p[1], s3s4_lat, s3s4_lng)) for i, p in enumerate(points)]
        s3s4_dists.sort(key=lambda x: x[1])
        s3s4_idx = s3s4_dists[0][0]
        print(f"  S3/S4 경계 가장 가까운 포인트: pt{s3s4_idx} (거리={s3s4_dists[0][1]:.1f}m)")
        print(f"  S4 구간 포인트 수: {len(points) - s3s4_idx}")
        print()
        print(f"  {'pt':>5}  {'lat':>12}  {'lng':>13}  {'t(s)':>8}  {'spd':>7}  {'cumDist':>10}  {'finDist':>9}")
        finish_lat, finish_lng = INJE_FINISH
        for i in range(s3s4_idx, len(points)):
            p = points[i]
            fdist = flat_dist(p[0], p[1], finish_lat, finish_lng)
            print(f"  {i:5d}  {p[0]:12.6f}  {p[1]:13.6f}  {p[2]/1000:8.2f}  "
                  f"{p[3]:6.1f}  {cum_dist[i]:10.1f}  {fdist:9.1f}")

        print()
        print("=== 델타 계산기 투영 시뮬레이션 (윈도우=50, back=10) ===")
        WINDOW = 50
        BACK   = 10
        MAX_DIST = 50.0
        last_seg = -1
        fail_count = 0

        cos_lat = math.cos(math.radians(points[0][0]))

        def seg_proj_dist(plat, plng, i):
            """포인트 p를 세그먼트 i에 투영한 거리"""
            if i >= len(points) - 1:
                return 1e9
            a = points[i];   b = points[i+1]
            # 도 → 미터 변환
            ax = a[1] * 111320 * cos_lat;  ay = a[0] * 111320
            bx = b[1] * 111320 * cos_lat;  by = b[0] * 111320
            px = plng * 111320 * cos_lat;  py = plat * 111320
            t = proj_t(px, py, ax, ay, bx, by)
            qx = ax + t*(bx-ax);  qy = ay + t*(by-ay)
            return math.sqrt((px-qx)**2 + (py-qy)**2)

        print(f"  {'pt':>5}  {'best_dist':>9}  {'best_seg':>8}  {'search':>14}  {'fail':>5}")
        for i in range(len(points)):
            p = points[i]
            if last_seg >= 0:
                s_start = max(0, last_seg - BACK)
                s_end   = min(last_seg + WINDOW, len(points) - 1)
            else:
                s_start = 0
                s_end   = len(points) - 1

            best_d   = 1e9
            best_seg = -1
            for s in range(s_start, s_end):
                d = seg_proj_dist(p[0], p[1], s)
                if d < best_d:
                    best_d = d
                    best_seg = s

            # fallback full search if needed
            fallback = False
            if best_d >= MAX_DIST:
                for s in range(0, len(points) - 1):
                    d = seg_proj_dist(p[0], p[1], s)
                    if d < best_d:
                        best_d = d
                        best_seg = s
                fallback = True

            if i >= s3s4_idx:  # S4 구간만 출력
                fail = "FAIL" if best_d >= MAX_DIST else ""
                fb   = "(FB)" if fallback else ""
                print(f"  {i:5d}  {best_d:9.1f}  {best_seg:8d}  [{s_start:5d},{s_end:5d}]  "
                      f"{fail:5s} {fb}")
                if best_d >= MAX_DIST:
                    fail_count += 1

            if best_d < MAX_DIST:
                last_seg = best_seg

        print(f"\n  S4 투영 실패 횟수: {fail_count}/{len(points) - s3s4_idx}")

def main():
    parser = argparse.ArgumentParser(description="GPS Laptimer 랩 데이터 파서")
    parser.add_argument("file", help="bin 파일 경로")
    parser.add_argument("--sector-check", action="store_true",
                        help="인제스피디움 섹터 경계 분석 추가")
    args = parser.parse_args()

    points, total_ms = parse_bin(args.file)
    analyze(points, total_ms, do_sector_check=args.sector_check)

if __name__ == "__main__":
    main()
