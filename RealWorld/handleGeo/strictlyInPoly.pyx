#cython: language_level=3

cimport cython

# Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'
cdef unsigned int onSegment(double p_0, double p_1, double q_0, double q_1, double r_0, double r_1):
    cdef unsigned int output = 0
    if max(p_0, r_0) >= q_0 >= min(p_0, r_0) and max(p_1, r_1) >= q_1 >= min(p_1, r_1):
        output = 1
    return output


cdef unsigned int orientation(double p_0, double p_1, double q_0, double q_1, double r_0, double r_1):
    cdef double val
    cdef unsigned int output
    val = (q_1 - p_1) * (r_0 - q_0) - (q_0 - p_0) * (r_1 - q_1)
    if val == 0.0:
        output = 0  # colinear
    elif val > 0.0:
        output = 1  # clock wise
    else:
        output = 2  # counterclock wise
    return output


# The function that returns true if line segment 'p1q1' and 'p2q2' intersect.
cdef unsigned int doIntersect(double p1_0, double p1_1, double q1_0, double q1_1, double p2_0, double p2_1, double q2_0, double q2_1):
    cdef unsigned int o1, o2, o3, o4
    cdef unsigned int output = 0

    # Find the four orientations needed for general and special cases
    o1 = orientation(p1_0, p1_1, q1_0, q1_1, p2_0, p2_1)
    o2 = orientation(p1_0, p1_1, q1_0, q1_1, q2_0, q2_1)
    o3 = orientation(p2_0, p2_1, q2_0, q2_1, p1_0, p1_1)
    o4 = orientation(p2_0, p2_1, q2_0, q2_1, q1_0, q1_1)

    # Define output
    if \
            (o1 != o2 and o3 != o4) or \
            (o1 == 0 and onSegment(p1_0, p1_1, p2_0, p2_1, q1_0, q1_1)) or \
            (o2 == 0 and onSegment(p1_0, p1_1, q2_0, q2_1, q1_0, q1_1)) or \
            (o3 == 0 and onSegment(p2_0, p2_1, p1_0, p1_1, q2_0, q2_1)) or \
            (o4 == 0 and onSegment(p2_0, p2_1, q1_0, q1_1, q2_0, q2_1)):
        output = 1

    return output


@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing
# Returns true if the point p lies inside the polygon[] with n vertices
cdef unsigned int check(double p_0, double p_1, double[:, :] coords):
    # Create a point for line segment from p to infinite
    cdef double *extreme = [9999999999999, p_1]

    # Count intersections of the above line with sides of polygon
    cdef unsigned int count = 0
    cdef unsigned int output = 0
    cdef Py_ssize_t i = 0
    cdef Py_ssize_t next
    cdef unsigned int condition = 1
    cdef unsigned int vertices = coords.shape[0]

    while condition == 1:
        next = (i + 1) % vertices

        # Check if the line segment from 'p' to 'extreme' intersects
        # with the line segment from 'polygon[i]' to 'polygon[next]'
        if doIntersect(coords[i][0], coords[i][1], coords[next][0], coords[next][1], p_0, p_1, extreme[0], extreme[1]):
            # If the point 'p' is colinear with line segment 'i-next',
            # then check if it lies on segment. If it lies, return true,
            # otherwise false
            if orientation(coords[i][0], coords[i][1], p_0, p_1, coords[next][0], coords[next][1]) == 0:
                return onSegment(coords[i][0], coords[i][1], p_0, p_1, coords[next][0], coords[next][1])
            count += 1

        i = next

        if i != 0:
            condition = 1
        else:
            condition = 0

    # Return true if count is odd, false otherwise
    if count % 2 == 1:
        output = 1

    return output


@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing
cdef unsigned int all_zero(double[:, :, :] array):
    cdef unsigned int output = 1
    cdef Py_ssize_t i_max = array.shape[0]
    cdef Py_ssize_t j_max = array.shape[1]
    cdef Py_ssize_t h_max = array.shape[2]
    cdef Py_ssize_t i, j, h
    for i in range(i_max):
        for j in range(j_max):
            for h in range(h_max):
                if array[i][j][h] != 0:
                    output = 0
                    return output
    return output


@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing
def strictlyInPoly(
        unsigned int[:, :] megaNodes,
        double xMin,
        double yMin,
        double xBoxMax,
        double xBoxMin,
        double yBoxMax,
        double yBoxMin,
        double nodeDistance,
        double nodeIntervalOffset,
        double shiftX,
        double shiftY,
        double[:, :] polygonCoordinates,
        unsigned int xNodes,
        unsigned int yNodes,
        double[:, :, :] cartObst
):

    cdef unsigned int megaNodesCount = 0
    cdef unsigned int len_cartObst, are_all_zero
    cdef Py_ssize_t i, j, k
    cdef double aux_0, aux_1
    cdef double a_0, a_1, b_0, b_1, c_0, c_1, d_0, d_1

    for i in range(xNodes):
        aux_0 = xMin + i * nodeDistance + shiftX
        for j in range(yNodes):
            aux_1 = yMin + j * nodeDistance + shiftY
            a_0 = aux_0 - nodeIntervalOffset
            a_1 = aux_1 + nodeIntervalOffset
            b_0 = aux_0 + nodeIntervalOffset
            b_1 = aux_1 + nodeIntervalOffset
            c_0 = aux_0 - nodeIntervalOffset
            c_1 = aux_1 - nodeIntervalOffset
            d_0 = aux_0 + nodeIntervalOffset
            d_1 = aux_1 - nodeIntervalOffset

            if \
                    check(a_0, a_1, polygonCoordinates) and \
                    check(b_0, b_1, polygonCoordinates) and \
                    check(c_0, c_1, polygonCoordinates) and \
                    check(d_0, d_1, polygonCoordinates):

                are_all_zero = all_zero(cartObst)
                if are_all_zero == 0:
                    len_cartObst = cartObst.shape[0]
                    k = 0
                    while megaNodes[i][j] != 1 and k < len_cartObst:
                        if \
                                check(a_0, a_1, cartObst[k]) or \
                                check(b_0, b_1, cartObst[k]) or \
                                check(c_0, c_1, cartObst[k]) or \
                                check(d_0, d_1, cartObst[k]):
                            megaNodes[i][j] = 1
                        else:
                            megaNodesCount += 1
                        k += 1

                if megaNodes[i][j] == 0:
                    # check maxs and mins
                    xBoxMax = max(xBoxMax, b_0, d_0)  # TODO only if nodeIntervalOffset > 0, else also a_0 and c_0
                    xBoxMin = min(xBoxMin, a_0, c_0)  # TODO only if nodeIntervalOffset > 0, else also b_0 and d_0
                    yBoxMax = max(yBoxMax, a_1, b_1)  # TODO idem
                    yBoxMin = min(yBoxMin, c_1, d_1)  # TODO idem

                    megaNodesCount += 1  # free space

            else:
                megaNodes[i][j] = 1  # Obstacle

    return megaNodesCount, xBoxMax, xBoxMin, yBoxMax, yBoxMin
