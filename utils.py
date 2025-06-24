import math

METERS_IN_DEGREE = 111319.5  # Approximate meters per degree latitude

def gps_to_local_xy(lat, lon, origin_lat, origin_lon):
   
    meters_in_long_degree = math.cos(math.radians(origin_lat)) * METERS_IN_DEGREE

    x = (lon - origin_lon) * meters_in_long_degree
    y = (lat - origin_lat) * METERS_IN_DEGREE

    return float(x), float(y)
