from geopy.distance import geodesic, distance
from geopy.point import Point
import math

# Constants for the WGS-84 ellipsoid
EARTH_RADIUS_EQUATOR = 6378137.0  # in meters
EARTH_RADIUS_POLAR = 6356752.314245  # in meters
EARTH_FLATTENING = (EARTH_RADIUS_EQUATOR - EARTH_RADIUS_POLAR) / EARTH_RADIUS_EQUATOR

# Explicitly specifying the ellipsoid
ELLIPSOID = 'WGS-84'

def add_distance_to_lat_lon(lat, lon, distance_meters_north, distance_meters_east):
    
    # Convert latitude and longitude to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    # Radius of the Earth at the given latitude
    sin_lat = math.sin(lat_rad)
    earth_radius_at_lat = EARTH_RADIUS_EQUATOR * (1 - EARTH_FLATTENING) / (1 - (2 * EARTH_FLATTENING - EARTH_FLATTENING**2) * sin_lat**2)**0.5

    # Calculate the new latitude
    delta_lat = distance_meters_north / EARTH_RADIUS_POLAR  # Use polar radius for northward movement
    new_lat_rad = lat_rad + delta_lat

    # Calculate the new longitude
    delta_lon = distance_meters_east / (earth_radius_at_lat * math.cos(lat_rad))
    new_lon_rad = lon_rad + delta_lon

    # Convert back to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    return new_lat, new_lon

def calculate_distance_north_east(lat1, lon1, lat2, lon2):
    
    # Convert degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Radius of the Earth at the given latitude (based on ellipsoid)
    sin_lat = math.sin(lat1_rad)
    earth_radius_at_lat = EARTH_RADIUS_EQUATOR * (1 - EARTH_FLATTENING) / (1 - (2 * EARTH_FLATTENING - EARTH_FLATTENING**2) * sin_lat**2)**0.5
    
    # Distance north (change in latitude)
    delta_lat = lat2_rad - lat1_rad
    dist_north = delta_lat * EARTH_RADIUS_POLAR  # Approximation using polar radius
    
    # Distance east (change in longitude, accounting for latitude)
    delta_lon = lon2_rad - lon1_rad
    dist_east = delta_lon * earth_radius_at_lat * math.cos(lat1_rad)
    
    return dist_north, dist_east

def decimal_degrees_to_degrees_minutes(decimal_degrees):
    """
    Converts decimal degrees to degrees and decimal minutes.
    
    :param decimal_degrees: Latitude or longitude in decimal degrees.
    :return: A tuple of (degrees, minutes) where degrees is an integer
             and minutes is a float.
    """
    degrees = int(decimal_degrees)
    minutes = (abs(decimal_degrees) - abs(degrees)) * 60
    return degrees, minutes

def add_body_frame_pos_to_lat_lon(lat, lon, x, y, z, roll, pitch, heading):
    """
    Adjusts a given latitude and longitude based on body frame position offsets and the ship's orientation.
    Accounts for the fact that the body frame's Z-axis is positive downward, X-axis is positive forward,
    and Y-axis is positive to the starboard side of the ship.

    :param lat: Initial latitude in decimal degrees.
    :param lon: Initial longitude in decimal degrees.
    :param x: Displacement along the ship's longitudinal axis in meters.
    :param y: Displacement along the ship's lateral axis in meters.
    :param z: Displacement along the ship's vertical axis in meters (positive downwards).
    :param roll: Ship's roll in degrees.
    :param pitch: Ship's pitch in degrees.
    :param heading: Ship's heading in degrees (from North).
    :return: Tuple of (new_latitude, new_longitude) in decimal degrees.
    """
    # Convert angles from degrees to radians
    heading_rad = math.radians(heading)
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)

    # Correct x and y for the roll and pitch first, considering small angles
    corrected_x = x + z * math.sin(pitch_rad)
    corrected_y = y + z * math.sin(roll_rad)

    # Calculate northward and eastward displacement considering the heading
    northward_displacement = corrected_x * math.cos(heading_rad) + corrected_y * math.sin(heading_rad)
    eastward_displacement = corrected_y * math.cos(heading_rad) - corrected_x * math.sin(heading_rad)

    # Calculate the new global position based on the displacements
    original_point = geodesic(meters=northward_displacement, ellipsoid=ELLIPSOID).destination((lat, lon), 0)  # North
    final_point = geodesic(meters=eastward_displacement, ellipsoid=ELLIPSOID).destination(original_point, 90)  # East

    return final_point.latitude, final_point.longitude
