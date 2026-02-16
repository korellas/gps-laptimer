#!/usr/bin/env python3
"""
Find closest track points in everland_track_data.h for given coordinates.
"""

import math
import re

# Target coordinates (int32 format: degrees * 1e7)
targets = [
    {"name": "Point 1", "lat": 372974004, "lon": 1272175125},
    {"name": "Point 2", "lat": 372961782, "lon": 1272138076},
]

def parse_track_data(filename):
    """Parse track points from the header file."""
    points = []
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # Find all track points in the format {lat, lon, timeMs, speedX10}
    # Pattern matches: {372962488, 1272065161, 50, 1525}
    pattern = r'\{(-?\d+),\s*(-?\d+),\s*(\d+),\s*(\d+)\}'
    matches = re.findall(pattern, content)
    
    for i, match in enumerate(matches):
        lat = int(match[0])
        lon = int(match[1])
        time_ms = int(match[2])
        speed_x10 = int(match[3])
        points.append({
            'index': i,
            'lat': lat,
            'lon': lon,
            'time_ms': time_ms,
            'speed_x10': speed_x10
        })
    
    return points

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees, int32 format)
    Returns distance in meters.
    """
    # Convert from int32 (degrees * 1e7) to radians
    lat1_rad = math.radians(lat1 / 1e7)
    lon1_rad = math.radians(lon1 / 1e7)
    lat2_rad = math.radians(lat2 / 1e7)
    lon2_rad = math.radians(lon2 / 1e7)
    
    # Haversine formula
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    # Radius of earth in meters
    r = 6371000
    
    return c * r

def calculate_cumulative_distances(points):
    """Calculate cumulative distance along the track."""
    cumulative_dist = [0.0]  # First point has 0 distance
    
    for i in range(1, len(points)):
        dist = haversine_distance(
            points[i-1]['lat'], points[i-1]['lon'],
            points[i]['lat'], points[i]['lon']
        )
        cumulative_dist.append(cumulative_dist[-1] + dist)
    
    return cumulative_dist

def find_closest_point(target_lat, target_lon, points):
    """Find the index of the closest point to the target coordinates."""
    min_dist = float('inf')
    closest_idx = -1
    
    for i, point in enumerate(points):
        dist = haversine_distance(target_lat, target_lon, point['lat'], point['lon'])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    return closest_idx, min_dist

def main():
    # Parse track data
    print("Parsing track data from everland_track_data.h...")
    points = parse_track_data('examples/everland_track_data.h')
    print(f"Total points parsed: {len(points)}")
    
    # Calculate cumulative distances
    print("\nCalculating cumulative distances...")
    cumulative_dist = calculate_cumulative_distances(points)
    
    # Find closest points for each target
    print("\n" + "="*70)
    print("RESULTS")
    print("="*70)
    
    for target in targets:
        idx, dist_to_target = find_closest_point(target['lat'], target['lon'], points)
        point = points[idx]
        cum_dist = cumulative_dist[idx]
        
        print(f"\n{target['name']}: lat={target['lat']}, lon={target['lon']}")
        print(f"  Closest point index: {idx}")
        print(f"  Distance to target:  {dist_to_target:.2f} meters")
        print(f"  Cumulative distance: {cum_dist:.2f} meters ({cum_dist/1000:.3f} km)")
        print(f"  Point details: lat={point['lat']}, lon={point['lon']}, timeMs={point['time_ms']}, speedX10={point['speed_x10']}")
    
    # Also print the lap boundaries for reference
    print("\n" + "="*70)
    print("LAP BOUNDARIES (for reference)")
    print("="*70)
    lap_boundaries = [
        (0, 1327, "Lap 5"),
        (1328, 2687, "Lap 3"),
        (2688, 4047, "Lap 4")
    ]
    for start, end, name in lap_boundaries:
        lap_dist = cumulative_dist[end] - cumulative_dist[start]
        print(f"{name}: indices {start}-{end}, distance: {lap_dist:.2f}m ({lap_dist/1000:.3f}km)")
    
    # Print total track distance
    print(f"\nTotal track distance (all laps): {cumulative_dist[-1]:.2f}m ({cumulative_dist[-1]/1000:.3f}km)")

if __name__ == '__main__':
    main()
