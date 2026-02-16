# Track Data Format

**SSOT:** This is the single source of truth for track data format and conventions.

## 1. Overview

This directory contains reference documentation for all supported tracks.

Each track has:
- Track geometry (finish line, sectors)
- Reference lap data
- Expected performance metrics
- GPS considerations

## 2. Track Files

| Track | File | Status |
|-------|------|--------|
| **Everland** | [everland.md](everland.md) | ✅ Active |
| **Yongin (future)** | yongin.md | 📋 Planned |
| **Inje (future)** | inje.md | 📋 Planned |
| **Taeback BMW (future)** | taeback.md | 📋 Planned |

## 3. Track Data Structure

### 3.1 Required Fields

Every track must define:

```cpp
const TrackLayout TRACK_NAME = {
    .name = "Track Name",

    // Finish line segment (two endpoints)
    .finishLat1 = 37.xxxxxx,
    .finishLng1 = 127.xxxxxx,
    .finishLat2 = 37.xxxxxx,
    .finishLng2 = 127.xxxxxx,

    // Valid crossing direction (heading range)
    .finishHeadingMin = 315.0f,  // Degrees (0-359)
    .finishHeadingMax = 45.0f,   // Can wrap around 0° (e.g., 315-45)

    // Sector configuration
    .numSectors = 3,
    .sectorBoundaries = {
        {lat1, lng1, "S1→S2"},
        {lat2, lng2, "S2→S3"},
        // ... (numSectors-1 boundaries)
    }
};
```

### 3.2 Coordinate Conventions

| Format | Value |
|--------|-------|
| **Datum** | WGS84 |
| **Units** | Decimal degrees |
| **Precision** | 7-8 decimal places (≤10cm) |
| **Order** | `lat, lng` (not `lng, lat`) |
| **Hemisphere** | Positive for N/E, negative for S/W |

**Example:**
```cpp
// Correct
.finishLat1 = 37.296096,   // 37.296096°N
.finishLng1 = 127.206860,  // 127.206860°E

// Incorrect (reversed)
.finishLat1 = 127.206860,  // Wrong!
.finishLng1 = 37.296096,   // Wrong!
```

### 3.3 Heading Convention

```
         0° (North)
            ↑
            |
270° ←------+-----→ 90° (East)
  (West)    |
            ↓
         180° (South)
```

**Heading Range:**
- Single range: `min=270, max=360` (westward crossing)
- Wrapped range: `min=315, max=45` (northward, crossing 0°)
- Measured clockwise from North
- Range covers valid approach directions

**Valid Heading Check:**
```cpp
bool isHeadingValid(float heading, float min, float max) {
    if (min <= max) {
        return (heading >= min && heading <= max);
    } else {
        // Wrapped around 0°
        return (heading >= min || heading <= max);
    }
}
```

## 4. Reference Lap Format

### 4.1 GPSPoint Array

Reference lap data is stored as a C array:

```cpp
// examples/TRACKNAME_reference_LAPTIME.h
const GPSPoint trackReference[] = {
    // {lat,      lng,       gpsTimeMs, lapTimeMs, speedKmh, headingDeg}
    {37.296096, 127.206860, 0,         0,         0.0,      0.0},      // Start
    {37.296123, 127.206875, 50,        50,        15.2,     12.5},     // +50ms
    {37.296145, 127.206892, 100,       100,       28.7,     14.8},     // +100ms
    // ... (1000-2000 points for typical lap)
};

const int trackReferenceCount = sizeof(trackReference) / sizeof(GPSPoint);
```

### 4.2 Naming Convention

**File:** `examples/TRACKNAME_reference_LAPTIME.h`

Examples:
- `everland_reference_5283.h` → 52.83 second lap
- `yongin_reference_12345.h` → 123.45 second lap

**LAPTIME format:** 4-5 digits, no decimal point
- `5283` = 52.83s
- `12345` = 123.45s

### 4.3 Data Quality Requirements

| Parameter | Requirement |
|-----------|-------------|
| **Sample Rate** | 10-25 Hz (40-100ms intervals) |
| **Fix Type** | 3D fix (minimum 6 satellites) |
| **Point Count** | 500-2000 points (depends on lap time) |
| **Coordinate Precision** | 7-8 decimal places |
| **Speed Range** | 0-250 km/h (realistic for track) |
| **Heading Range** | 0-359 degrees |

## 5. Sector Configuration

### 5.1 Sector Count

Recommended sector count:

| Track Length | Sectors | Reason |
|--------------|---------|--------|
| < 3 km | 2-3 | Short track, simple analysis |
| 3-10 km | 3-4 | Medium track, balanced detail |
| > 10 km | 4-6 | Long track, detailed analysis |

**Maximum:** `MAX_SECTORS = 8` (defined in `config.h`)

### 5.2 Sector Boundary Placement

**Guidelines:**
- Place at natural track divisions (corners, straights)
- Avoid ambiguous locations (mid-corner)
- Ensure clear GPS visibility (no tunnels/trees)
- Roughly equal sector lengths (±30%)

**Good boundary locations:**
- End of main straight
- Apex of major corner
- Start of uphill/downhill section

**Bad boundary locations:**
- Inside tunnel
- Under dense tree cover
- Mid-corner (hard to identify)
- Too close to finish line (<100m)

### 5.3 Boundary Coordinates

Sector boundaries are **points**, not lines:

```cpp
.sectorBoundaries = {
    {37.2974004, 127.2175125, "S1→S2"},  // Single coordinate
    {37.2961782, 127.2138076, "S2→S3"},  // Single coordinate
}
```

**Detection:**
- System finds closest point in reference lap to boundary coordinate
- Uses cumulative distance of that point as threshold
- Triggers when `trackDistanceM >= boundary distance`
- Tolerance: ±10-20 meters (depends on GPS accuracy)

## 6. Track Validation

### 6.1 Validation Checklist

Before adding a new track:

- [ ] Finish line coordinates are correct (verified on map)
- [ ] Heading range covers valid crossing direction
- [ ] Sector boundaries are GPS-visible locations
- [ ] Reference lap data is clean (no spikes, gaps)
- [ ] Lap time is realistic for track
- [ ] Cumulative distances are monotonic increasing
- [ ] Track length matches expected value (±5%)

### 6.2 Testing Procedure

```cpp
// 1. Load track definition
const TrackLayout* track = &NEW_TRACK;

// 2. Load reference lap
loadReferenceLap("examples/newtrack_reference_XXXX.h");

// 3. Verify sector distances
updateSectorDistancesFromReference();
// Check logs for boundary distances

// 4. Simulate lap
setMode(GPSMode::SIMULATION);
// Verify sector transitions occur at expected locations

// 5. Check finish line detection
// Verify lap completes when crossing finish line
```

## 7. CSV Export/Import (Future)

Planned CSV format for track templates:

```csv
name,center_lat,center_lng,radius_m,finish_lat1,finish_lng1,finish_lat2,finish_lng2,heading_min,heading_max,num_sectors
Everland,37.2962,127.2100,500,37.296096,127.206860,37.296159,127.206621,315,45,3
```

Sector boundaries in separate CSV:
```csv
track_name,sector_num,boundary_lat,boundary_lng,name
Everland,1,37.2974004,127.2175125,S1→S2
Everland,2,37.2961782,127.2138076,S2→S3
```

**See:** `spec.md` for full CSV template specification

## 8. Adding a New Track

### Step 1: Collect Data
```bash
# Record GPS data on track (multiple laps)
# Select best lap as reference
# Export to GPSPoint array
```

### Step 2: Define Track
```cpp
// components/track/builtin_tracks.h
const TrackLayout NEW_TRACK = {
    .name = "New Track",
    .finishLat1 = ...,
    .finishLng1 = ...,
    // ... (fill in all fields)
};
```

### Step 3: Create Reference File
```cpp
// examples/newtrack_reference_XXXX.h
const GPSPoint newtrackReference[] = {
    // ... (paste cleaned GPS data)
};
```

### Step 4: Document
```markdown
# docs/reference/tracks/newtrack.md
# New Track Specification
...
```

### Step 5: Test
```bash
# Simulate and verify
idf.py build flash monitor
# Switch to simulation mode, load track
```

## 9. Data Sources

GPS data can be obtained from:

| Source | Format | Quality |
|--------|--------|---------|
| **u-blox GPS** | UBX binary | Best (native) |
| **NMEA Logger** | NMEA text | Good (standard) |
| **GPX File** | XML | Moderate (common) |
| **RaceCapture** | CSV/JSON | Good (racing app) |
| **Phone GPS** | Various | Moderate (lower accuracy) |

**Recommended:** u-blox GPS module (10 Hz, binary protocol)

## 10. Coordinate Conversion Tools

**Online Tools:**
- Google Maps: Right-click → "What's here?"
- GPS Visualizer: https://www.gpsvisualizer.com/
- GPSBabel: Format conversion

**Verification:**
```bash
# Python script to validate coordinates
python tools/validate_track.py examples/newtrack_reference.h
```

## References

- **Example Track:** [everland.md](everland.md)
- **Data Structures:** [../data-structures.md](../data-structures.md)
- **Architecture:** [../architecture.md](../architecture.md)
- **Code:** `components/track/builtin_tracks.h`
