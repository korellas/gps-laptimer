/**
 * @file geo_utils.h
 * @brief Geographic calculation utilities
 * @version 1.0
 * 
 * Functions for GPS coordinate calculations including distance,
 * bearing, and projection operations.
 */

#ifndef GEO_UTILS_H
#define GEO_UTILS_H

#include <cmath>

// ============================================================
// CONSTANTS
// ============================================================

constexpr double GEO_EARTH_RADIUS_M = 6371000.0;    // Earth radius in meters
constexpr double GEO_DEG_TO_RAD = M_PI / 180.0;     // Degrees to radians
constexpr double GEO_RAD_TO_DEG = 180.0 / M_PI;     // Radians to degrees

// ============================================================
// DISTANCE CALCULATIONS
// ============================================================

/**
 * @brief Calculate distance between two GPS coordinates using Haversine formula
 * 
 * @param lat1 Latitude of first point (degrees)
 * @param lng1 Longitude of first point (degrees)
 * @param lat2 Latitude of second point (degrees)
 * @param lng2 Longitude of second point (degrees)
 * @return Distance in meters
 */
float haversineDistanceMeters(double lat1, double lng1, double lat2, double lng2);

/**
 * @brief Fast approximate distance for nearby points (< 1km)
 * 
 * Uses equirectangular approximation which is faster but less accurate
 * for large distances. Good enough for track segment calculations.
 * 
 * @param lat1 Latitude of first point (degrees)
 * @param lng1 Longitude of first point (degrees)
 * @param lat2 Latitude of second point (degrees)
 * @param lng2 Longitude of second point (degrees)
 * @return Approximate distance in meters
 */
float fastDistanceMeters(double lat1, double lng1, double lat2, double lng2);

/**
 * @brief Ultra-fast distance with pre-computed cos(lat)
 *
 * For hot-path usage where cos(latitude) can be computed once
 * and reused across many distance calculations (e.g., segment search).
 * Uses float-only arithmetic for ESP32-S3 FPU optimization.
 *
 * @param lat1 Latitude of first point (degrees)
 * @param lng1 Longitude of first point (degrees)
 * @param lat2 Latitude of second point (degrees)
 * @param lng2 Longitude of second point (degrees)
 * @param cosLat Pre-computed cos(latitude) in radians
 * @return Approximate distance in meters
 */
inline float fastDistanceMetersPrecomp(double lat1, double lng1,
                                        double lat2, double lng2,
                                        float cosLat) {
    float dx = (float)(lng2 - lng1) * cosLat * 111320.0f;
    float dy = (float)(lat2 - lat1) * 111320.0f;
    return sqrtf(dx * dx + dy * dy);
}

// ============================================================
// BEARING / HEADING
// ============================================================

/**
 * @brief Calculate initial bearing from point 1 to point 2
 * 
 * @param lat1 Latitude of start point (degrees)
 * @param lng1 Longitude of start point (degrees)
 * @param lat2 Latitude of end point (degrees)
 * @param lng2 Longitude of end point (degrees)
 * @return Bearing in degrees (0-360, 0=North, 90=East)
 */
float calculateBearing(double lat1, double lng1, double lat2, double lng2);

/**
 * @brief Calculate the angular difference between two bearings
 * 
 * @param bearing1 First bearing (degrees)
 * @param bearing2 Second bearing (degrees)
 * @return Smallest angle difference (0-180 degrees)
 */
float bearingDifference(float bearing1, float bearing2);

/**
 * @brief Check if a heading is within a valid range
 * 
 * Handles wrap-around at 0/360 degrees.
 * 
 * @param heading Current heading (degrees)
 * @param minHeading Minimum valid heading (degrees)
 * @param maxHeading Maximum valid heading (degrees)
 * @return true if heading is within range
 */
bool isHeadingInRange(float heading, float minHeading, float maxHeading);

// ============================================================
// PROJECTION
// ============================================================

/**
 * @brief Project a point onto a line segment
 * 
 * Finds the closest point on line segment p1-p2 to point p.
 * Returns the interpolation factor t where:
 *   - t = 0.0 means projection is at p1
 *   - t = 1.0 means projection is at p2
 *   - t = 0.5 means projection is at midpoint
 * 
 * @param px Latitude of point to project
 * @param py Longitude of point to project
 * @param p1x Latitude of segment start
 * @param p1y Longitude of segment start
 * @param p2x Latitude of segment end
 * @param p2y Longitude of segment end
 * @return Interpolation factor t, clamped to [0, 1]
 */
float projectToSegment(double px, double py, 
                       double p1x, double p1y, 
                       double p2x, double p2y);

/**
 * @brief Calculate the projected point coordinates
 * 
 * Given segment p1-p2 and interpolation factor t, calculate the
 * actual coordinates of the projected point.
 * 
 * @param p1Lat Latitude of segment start
 * @param p1Lng Longitude of segment start
 * @param p2Lat Latitude of segment end
 * @param p2Lng Longitude of segment end
 * @param t Interpolation factor (0-1)
 * @param outLat Output: projected point latitude
 * @param outLng Output: projected point longitude
 */
void interpolatePoint(double p1Lat, double p1Lng,
                      double p2Lat, double p2Lng,
                      float t,
                      double& outLat, double& outLng);

// ============================================================
// LINE INTERSECTION
// ============================================================

/**
 * @brief Check if two line segments intersect
 * 
 * Used for finish line crossing detection.
 * 
 * @param p1Lat Start of first segment - latitude
 * @param p1Lng Start of first segment - longitude
 * @param p2Lat End of first segment - latitude
 * @param p2Lng End of first segment - longitude
 * @param q1Lat Start of second segment - latitude
 * @param q1Lng Start of second segment - longitude
 * @param q2Lat End of second segment - latitude
 * @param q2Lng End of second segment - longitude
 * @return true if segments intersect
 */
bool segmentsIntersect(double p1Lat, double p1Lng,
                       double p2Lat, double p2Lng,
                       double q1Lat, double q1Lng,
                       double q2Lat, double q2Lng);

// ============================================================
// POSITION ESTIMATION
// ============================================================

/**
 * @brief Estimate new position given starting point, speed, heading, and time
 * 
 * Used for dead reckoning when GPS signal is lost.
 * 
 * @param lat Starting latitude (degrees)
 * @param lng Starting longitude (degrees)
 * @param speedKmh Speed in km/h
 * @param headingDeg Heading in degrees (0=North)
 * @param elapsedMs Time elapsed in milliseconds
 * @param outLat Output: estimated latitude
 * @param outLng Output: estimated longitude
 */
void estimatePosition(double lat, double lng,
                      float speedKmh, float headingDeg,
                      unsigned long elapsedMs,
                      double& outLat, double& outLng);

#endif // GEO_UTILS_H