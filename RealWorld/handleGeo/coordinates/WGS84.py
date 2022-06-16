import math


import math

from RealWorld.handleGeo.coordinates.ECEF import ECEF
from RealWorld.handleGeo.coordinates.NED import NED


class WGS84_class:

    def __init__(self, latitude, longitude, height):
        self.latitude = latitude
        self.longitude = longitude
        self.height = height


    def toString(self):
        return "{0:.6f}, {1:.6f}, {2:.2f}".format(math.degrees(self.latitude), math.degrees(self.longitude), self.height)



SEMI_MAJOR_AXIS = 6378137.0
FIRST_ECCENTRICITY_SQUARED = 0.00669437999013


def distance(pointA, pointB):
    ecefA = toECEF(pointA)
    ecefB = toECEF(pointB)

    x = ecefA.x - ecefB.x
    y = ecefA.y - ecefB.y
    z = ecefA.z - ecefB.z

    return math.sqrt(x * x + y * y + z * z)


def displacement(pointA, pointB):

    ecefA = toECEF(pointA)
    ecefB = toECEF(pointB)

    ox = ecefB.x - ecefA.x
    oy = ecefB.y - ecefA.y
    oz = ecefB.z - ecefA.z
    slat = math.sin(pointA.latitude)
    clat = math.cos(pointA.latitude)
    slon = math.sin(pointA.longitude)
    clon = math.cos(pointA.longitude)

    ned = NED(-slat * clon * ox - slat * slon * oy + clat * oz, -slon * ox + clon * oy,
              -clat * clon * ox - clat * slon * oy - slat * oz)
    return ned


#    *
#     * Displace a WGS-84 coordinate in the NED frame according to given offsets.
#     *
#     * @param wgs84 reference WGS-84 coordinate.
#     * @param ned   North-East-Down displacement.
#     * @return displaced WGS-84 coordinate.
#


def displace(wgs84, ned):
    ecef = toECEF(wgs84)

    # Compute Geocentric latitude.
    phi = math.atan2(ecef.z, math.sqrt(ecef.x * ecef.x + ecef.y * ecef.y))

    # Compute all needed sine and cosine terms for conversion.
    slon = math.sin(wgs84.longitude)
    clon = math.cos(wgs84.longitude)
    sphi = math.sin(phi)
    cphi = math.cos(phi)

    # Obtain ECEF coordinates of displaced point
    # Note: some signs from standard ENU formula
    # are inverted - we are working with NED (= END) coordinates
    ecef.x += -slon * ned.east - clon * sphi * ned.north - clon * cphi * ned.down
    ecef.y += clon * ned.east - slon * sphi * ned.north - slon * cphi * ned.down
    ecef.z += cphi * ned.north - sphi * ned.down

    # Convert back to WGS-84 coordinates.
    return fromECEF(ecef)


#    *
#     * Convert WGS-84 coordinates to ECEF (Earth Center Earth Fixed) coordinates.
#     *
#     * @param wgs84 WGS-84 coordinates.
#     *
#     * @return corresponding ECEF coordinates.
#


def toECEF(wgs84):
    cos_lat = math.cos(wgs84.latitude)
    sin_lat = math.sin(wgs84.latitude)
    cos_lon = math.cos(wgs84.longitude)
    sin_lon = math.sin(wgs84.longitude)
    rn = __computeRn(wgs84.latitude)

    ecef = ECEF((rn + wgs84.height) * cos_lat * cos_lon, (rn + wgs84.height) * cos_lat * sin_lon,
                (((1.0 - FIRST_ECCENTRICITY_SQUARED) * rn) + wgs84.height) * sin_lat)
    # ecef.x = (rn + wgs84.height) * cos_lat * cos_lon
    # ecef.y = (rn + wgs84.height) * cos_lat * sin_lon
    # ecef.z = (((1.0 - FIRST_ECCENTRICITY_SQUARED) * rn) + wgs84.height) * sin_lat
    return ecef


#    *
#     * Convert ECEF (x,y,z) to WGS-84 (lat, lon, hae).
#     *
#     * @param ecef ECEF coordinates.
#

def fromECEF(ecef):


    p = math.sqrt(ecef.x * ecef.x + ecef.y * ecef.y)
    latitude = math.atan2(ecef.z / p, 0.01)
    longitude = math.atan2(ecef.y, ecef.x)
    n = __computeRn(latitude)
    height = p / math.cos(latitude) - n

    wgs84 = WGS84_class(latitude, longitude, height)

    old_hae = -1e-9
    num = ecef.z / p

    while abs(wgs84.height - old_hae) > 1e-4:
        old_hae = wgs84.height
        den = 1 - FIRST_ECCENTRICITY_SQUARED * n / (n + wgs84.height)
        wgs84.latitude = math.atan2(num, den)
        n = __computeRn(wgs84.latitude)
        wgs84.height = p / math.cos(wgs84.latitude) - n

    return wgs84


def __computeRn(lat):
    lat_sin = math.sin(lat)

    return SEMI_MAJOR_AXIS / math.sqrt(1 - FIRST_ECCENTRICITY_SQUARED * (lat_sin * lat_sin))

