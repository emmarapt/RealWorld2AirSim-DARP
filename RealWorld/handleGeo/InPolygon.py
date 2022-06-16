import sys
import numpy as np
from numba import njit


class Point(object):

    def __init__(self, Lat, Lon):
        self.Lat = Lat
        self.Lon = Lon


# Returns true if the point p lies inside the polygon[] with n vertices
@njit
def check(point, coords):
    n = len(coords)
    polygon = np.zeros((n, 2))
    # polygon = [None for _ in range(n)]
    for i in range(n):
        # polygon[i] = Point(coords[i][0], coords[i][1])
        polygon[i, 0] = coords[i][0]
        polygon[i, 1] = coords[i][1]

    # p = Point(point[0], point[1])
    p = np.array([point[0], point[1]])

    # Create a point for line segment from p to infinite
    # extreme = Point(sys.float_info.max, p.Lon)
    extreme = np.array([9999999999999, p[1]])

    # Count intersections of the above line with sides of polygon
    count = 0
    i = 0
    condition = True
    while condition:
        next = (i + 1) % n

        # Check if the line segment from 'p' to 'extreme' intersects
        # with the line segment from 'polygon[i]' to 'polygon[next]'
        if doIntersect(polygon[i], polygon[next], p, extreme):
            # If the point 'p' is colinear with line segment 'i-next',
            # then check if it lies on segment. If it lies, return true,
            # otherwise false
            if orientation(polygon[i], p, polygon[next]) == 0:
                return onSegment(polygon[i], p, polygon[next])

            count += 1
        i = next
        condition = i != 0

    # Return true if count is odd, false otherwise
    return count % 2 == 1


# The function that returns true if line segment 'p1q1' and 'p2q2' intersect.
@njit
def doIntersect(p1, q1, p2, q2):
    # Find the four orientations needed for general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if o1 == 0 and onSegment(p1, p2, q1):
        return True

    # p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if o2 == 0 and onSegment(p1, q2, q1):
        return True

    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if o3 == 0 and onSegment(p2, p1, q2):
        return True

    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if o4 == 0 and onSegment(p2, q1, q2):
        return True

    return False  # Doesn't fall in any of the above cases


@njit
def orientation(p, q, r):
    # val = (q.Lon - p.Lon) * (r.Lat - q.Lat) - (q.Lat - p.Lat) * (r.Lon - q.Lon)
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0.0:
        return 0  # colinear
    return 1 if (val > 0.0) else 2  # clock or counterclock wise


# Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'

@njit
def onSegment(p, q, r):
    if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1],
                                                                                                       r[1]):
        return 1
    return False
