from math import radians, cos, sin, asin, sqrt, atan2, degrees, pi, log, tan
import time

def calcBearingToTarget(lat1, lon1, lat2, lon2):
    """
    Calculate the angle between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    dLon = lon2 - lon1
    dPhi = log((tan(lat2/2.0 + pi/4.0) / tan(lat1/2.0 + pi/4.0)))
    if abs(dLon) > pi:
        if (dLon) > 0.0:
            dLon = -(2.0 * pi - dLon)
        else:
            dLon = (2.0 * pi + dLon)

    bearing = (degrees(atan2(dLon, dPhi)) + 360.0) % 360.0
    return bearing

def haversineDistanceToTarget(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lat1, lon1, lat2, lon2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return (c * r) * 1000

lat1, lon1 = [-19.869951, -43.965057]
lat2, lon2 = [-19.869394, -43.964293]

print haversineDistanceToTarget(lat1, lon1, lat2, lon2)
print calcBearingToTarget(lat1, lon1, lat2, lon2)