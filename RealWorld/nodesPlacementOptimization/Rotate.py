import math
import numpy as np

from RealWorld.handleGeo.NodesInPoly import NodesInPoly
from RealWorld.helpers import MinMax

from numba import njit

class Rotate:

    # def __init__(self):
    #
    # 	self.optimalTheta = 0
    # 	self.range = 90

    def setParameters(self, optimalTheta, range):
        self.optimalTheta = optimalTheta
        self.range = range

    def setTheta(self, theta):
        self.optimalTheta = theta

    def setRange(self, range):
        self.range = range

    def findOptimalTheta(self, scanDist, cartUnrotated, cartObstUnrotated, stepSize, shiftX, shiftY):
        testRange = int((self.range / stepSize))

        nodesIn = [0 for _ in range(testRange)]
        area = [0 for _ in range(testRange)]

        for i in range(testRange):
            cartObstRotated = [[None for i in range(len(cartObstUnrotated))]]

            for j in range(len(cartObstUnrotated)):
                cartObstRotated[j] = self.rotatePolygon(cartObstUnrotated[j])

            testTheta = NodesInPoly(self.rotatePolygon(cartUnrotated), cartObstRotated, scanDist, True, True, shiftX,
                                    shiftY)
            nodesIn[i] = testTheta.getMegaNodesInCount()
            area[i] = testTheta.getBoundingBoxArea()
            self.optimalTheta = self.optimalTheta + stepSize

        theta = float(MinMax.indMaxNodeMinArea(nodesIn, area) * stepSize)
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" +
              "        Optimal Rotation Angle for Paths: ", theta, " degrees\n" +
              "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        self.optimalTheta = theta


    def rotatePolygon(self, cart):

        l = len(cart)
        m = len(cart[0])
        rotated = np.zeros((l, m)) #np.empty((l, m), np.float64)  # np.zeros((l, m))

        for i in range(l):
            rotated[i][0] = cart[i][0] * math.cos(math.radians(self.optimalTheta)) - cart[i][1] * math.sin(
                math.radians(self.optimalTheta))
            rotated[i][1] = cart[i][0] * math.sin(math.radians(self.optimalTheta)) + cart[i][1] * math.cos(
                math.radians(self.optimalTheta))

        return rotated

    def rotateBackWaypoints(self, iWaypoints):

        minusTheta = -self.optimalTheta

        l = len(iWaypoints)
        waypoints = []

        for i in range(l):
            a = iWaypoints[i][0] * math.cos(math.radians(minusTheta)) - iWaypoints[i][1] * math.sin(
                math.radians(minusTheta))
            b = iWaypoints[i][0] * math.sin(math.radians(minusTheta)) + iWaypoints[i][1] * math.cos(
                math.radians(minusTheta))
            waypoints.append([a, b])

        return waypoints
