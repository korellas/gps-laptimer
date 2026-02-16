/**
 * @file geo_utils.cpp
 * @brief Geographic calculation utilities implementation
 * @version 1.0
 */

#include "geo_utils.h"
#include <algorithm>

// ============================================================
// DISTANCE CALCULATIONS
// ============================================================

float haversineDistanceMeters(double lat1, double lng1, double lat2, double lng2) {
    double dLat = (lat2 - lat1) * GEO_DEG_TO_RAD;
    double dLng = (lng2 - lng1) * GEO_DEG_TO_RAD;
    
    double lat1Rad = lat1 * GEO_DEG_TO_RAD;
    double lat2Rad = lat2 * GEO_DEG_TO_RAD;
    
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1Rad) * cos(lat2Rad) *
               sin(dLng / 2.0) * sin(dLng / 2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return (float)(GEO_EARTH_RADIUS_M * c);
}

float fastDistanceMeters(double lat1, double lng1, double lat2, double lng2) {
    // Equirectangular approximation - float 전용 (ESP32-S3 FPU 활용)
    float avgLatRad = (float)((lat1 + lat2) * 0.5 * GEO_DEG_TO_RAD);
    float cosLat = cosf(avgLatRad);
    float dx = (float)(lng2 - lng1) * cosLat * 111320.0f;
    float dy = (float)(lat2 - lat1) * 111320.0f;
    return sqrtf(dx * dx + dy * dy);
}

// ============================================================
// BEARING / HEADING
// ============================================================

float calculateBearing(double lat1, double lng1, double lat2, double lng2) {
    double lat1Rad = lat1 * GEO_DEG_TO_RAD;
    double lat2Rad = lat2 * GEO_DEG_TO_RAD;
    double dLng = (lng2 - lng1) * GEO_DEG_TO_RAD;
    
    double x = sin(dLng) * cos(lat2Rad);
    double y = cos(lat1Rad) * sin(lat2Rad) - 
               sin(lat1Rad) * cos(lat2Rad) * cos(dLng);
    
    double bearing = atan2(x, y) * GEO_RAD_TO_DEG;
    
    // Normalize to 0-360
    if (bearing < 0) {
        bearing += 360.0;
    }
    
    return (float)bearing;
}

float bearingDifference(float bearing1, float bearing2) {
    float diff = fabs(bearing1 - bearing2);
    
    // Take the smaller angle
    if (diff > 180.0f) {
        diff = 360.0f - diff;
    }
    
    return diff;
}

bool isHeadingInRange(float heading, float minHeading, float maxHeading) {
    // Normalize heading to 0-360
    while (heading < 0) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    
    // Handle wrap-around case (e.g., min=350, max=10)
    if (minHeading <= maxHeading) {
        // Normal case: min < max
        return heading >= minHeading && heading <= maxHeading;
    } else {
        // Wrap-around case: range crosses 0/360
        return heading >= minHeading || heading <= maxHeading;
    }
}

// ============================================================
// PROJECTION
// ============================================================

float projectToSegment(double px, double py, 
                       double p1x, double p1y, 
                       double p2x, double p2y) {
    double dx = p2x - p1x;
    double dy = p2y - p1y;
    double lenSq = dx * dx + dy * dy;
    
    // Degenerate segment (start == end)
    if (lenSq < 1e-12) {
        return 0.0f;
    }
    
    // Calculate projection parameter t
    double t = ((px - p1x) * dx + (py - p1y) * dy) / lenSq;
    
    // Clamp to segment
    return std::clamp((float)t, 0.0f, 1.0f);
}

void interpolatePoint(double p1Lat, double p1Lng,
                      double p2Lat, double p2Lng,
                      float t,
                      double& outLat, double& outLng) {
    outLat = p1Lat + t * (p2Lat - p1Lat);
    outLng = p1Lng + t * (p2Lng - p1Lng);
}

// ============================================================
// LINE INTERSECTION
// ============================================================

bool segmentsIntersect(double p1Lat, double p1Lng,
                       double p2Lat, double p2Lng,
                       double q1Lat, double q1Lng,
                       double q2Lat, double q2Lng) {
    // Direction vectors
    double d1x = p2Lng - p1Lng;
    double d1y = p2Lat - p1Lat;
    double d2x = q2Lng - q1Lng;
    double d2y = q2Lat - q1Lat;
    
    // Cross product of directions
    double cross = d1x * d2y - d1y * d2x;
    
    // Parallel lines (or nearly parallel)
    if (fabs(cross) < 1e-12) {
        return false;
    }
    
    // Vector from p1 to q1
    double dx = q1Lng - p1Lng;
    double dy = q1Lat - p1Lat;
    
    // Calculate intersection parameters
    double t = (dx * d2y - dy * d2x) / cross;
    double u = (dx * d1y - dy * d1x) / cross;
    
    // Check if intersection is within both segments
    return (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0);
}

// ============================================================
// POSITION ESTIMATION
// ============================================================

void estimatePosition(double lat, double lng,
                      float speedKmh, float headingDeg,
                      unsigned long elapsedMs,
                      double& outLat, double& outLng) {
    // Convert speed to m/s
    double speedMs = speedKmh * 1000.0 / 3600.0;
    
    // Calculate distance traveled
    double distanceM = speedMs * (elapsedMs / 1000.0);
    
    // Convert heading to radians
    double headingRad = headingDeg * GEO_DEG_TO_RAD;
    
    // Calculate displacement in meters
    // North component (latitude direction)
    double dNorth = distanceM * cos(headingRad);
    // East component (longitude direction)
    double dEast = distanceM * sin(headingRad);
    
    // Convert to degrees
    // 1 degree latitude ≈ 111,320 meters
    double dLat = dNorth / 111320.0;
    
    // 1 degree longitude ≈ 111,320 * cos(latitude) meters
    double latRad = lat * GEO_DEG_TO_RAD;
    double dLng = dEast / (111320.0 * cos(latRad));
    
    outLat = lat + dLat;
    outLng = lng + dLng;
}
