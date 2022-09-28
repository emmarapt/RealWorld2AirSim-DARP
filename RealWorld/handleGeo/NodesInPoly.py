import math
import sys
import time

import numpy as np
from numba import njit

from RealWorld.ConnectComponent import ConnectComponent
from RealWorld.handleGeo import strictlyInPoly
from RealWorld.helpers import MinMax


class TimeItCount:
    def __init__(self, method):
        self.method = method
        self.total_time = 0
        self.total_runs = 0
        self.first = True

    def __call__(self, *args, **kw):
        ts = time.time()
        result = self.method(*args, **kw)
        te = time.time()
        elapsed_time = te - ts
        if not self.first:
            self.total_time += elapsed_time
            self.total_runs += 1
            print(
                f'{self.method.__name__}. Elapsed time {elapsed_time}. Total elapsed time {self.total_time}. Total runs {self.total_runs}. Mean {self.total_time / self.total_runs}')
        else:
            self.first = False
        return result


class NodesInPoly(object):

    def __init__(self, cartCoords, cartObst, scanDist, pathsStrictlyInPoly, hideInfo, shiftX, shiftY):

        self.scanDist = scanDist
        self.pathsStrictlyInPoly = pathsStrictlyInPoly
        self.hideInfo = hideInfo
        if len(cartObst)==0:
            self.cartObst = np.zeros((1,4,2))
        else:
            self.cartObst = np.array(cartObst)

        self.shiftX = shiftX
        self.shiftY = shiftY

        self.nodeDistance = 2 * self.scanDist
        self.nodeIntervalOffset = self.scanDist / 2
        self.polygonCoordinates = cartCoords


        self.xMax = MinMax.xMax(self.polygonCoordinates) + self.nodeDistance
        self.xMin = MinMax.xMin(self.polygonCoordinates) - self.nodeDistance
        self.yMax = MinMax.yMax(self.polygonCoordinates) + self.nodeDistance
        self.yMin = MinMax.yMin(self.polygonCoordinates) - self.nodeDistance

        self.xRngMeters = self.xMax - self.xMin
        self.yRngMeters = self.yMax - self.yMin

        self.xNodes = int((self.xRngMeters / self.nodeDistance))
        self.yNodes = int((self.yRngMeters / self.nodeDistance))

        # self.megaNodes = np.zeros((self.xNodes, self.yNodes, 3))
        # self.subNodes = np.zeros((2 * self.xNodes, 2 * self.yNodes, 3))

        if not hideInfo:
            print("Bounding box: ", self.xRngMeters, " x ", self.yRngMeters, " meters")
            print("xNodes: ", self.xNodes)
            print("yNodes: ", self.yNodes)
            print("Total number of mega-nodes: ", self.xNodes * self.yNodes)

        xInter = self.xMax - self.xMin - (self.xNodes - 1) * self.nodeDistance
        yInter = self.yMax - self.yMin - (self.yNodes - 1) * self.nodeDistance

        if self.pathsStrictlyInPoly:
            self.megaNodes, self.megaNodesCount, self.xBoxMax, self.xBoxMin, self.yBoxMax, self.yBoxMin = self.strictlyInPolyWrap(
                self.xMin,
                self.yMin,
                self.nodeDistance,
                self.hideInfo,
                self.nodeIntervalOffset,
                self.shiftX,
                self.shiftY,
                self.polygonCoordinates,
                self.xNodes,
                self.yNodes,
                self.cartObst,
                self.initialization,
                strictlyInPoly.strictlyInPoly
            )
        else:
            raise NotImplementedError()

    @staticmethod
    #@TimeItCount
    def strictlyInPolyWrap(xMin, yMin, nodeDistance, hideInfo, nodeIntervalOffset, shiftX, shiftY, polygonCoordinates, xNodes, yNodes, cartObst, initialization, computation):

        # initialization
        megaNodes = initialization(
            xNodes,
            yNodes
        )

        # computation
        megaNodesCount, xBoxMax, xBoxMin, yBoxMax, yBoxMin = computation(
            megaNodes,
            xMin,
            yMin,
            -sys.maxsize,
            sys.maxsize,
            -sys.maxsize,
            sys.maxsize,
            nodeDistance,
            nodeIntervalOffset,
            shiftX,
            shiftY,
            polygonCoordinates,
            xNodes,
            yNodes,
            cartObst
        )

        if not hideInfo:
            print("Number of mega-nodes that are used for STC: ", megaNodesCount)
            print("Number of sub-nodes that will be used for trajectories: ", 4.0 * megaNodesCount)
        return megaNodes, megaNodesCount, xBoxMax, xBoxMin, yBoxMax, yBoxMin

    @staticmethod
    @njit
    def initialization(xNodes, yNodes):
        return np.zeros((xNodes, yNodes), dtype=np.uintc)

    def getOptimizationIndex(self):

        a = 0.68
        b = 0.32
        c = 0.25

        polygonArea = self.getPolygonArea()

        nodesInTerm = (self.megaNodesCount * math.pow(self.nodeDistance, 2)) / polygonArea
        minBBAreaTerm = polygonArea / self.getBoundingBoxArea()

        if self.megaNodesCount > 0:
            equalMarginsTerm = self.marginNormSSI()

        else:
            equalMarginsTerm = 1

        optimizationIndex = a * nodesInTerm + b * minBBAreaTerm - c * equalMarginsTerm

        """ WITHOUT ConnectComponent"""
        G2G = ConnectComponent()

        connectivityTest = np.zeros((self.xNodes, self.yNodes))
        connectivityTest[:] = np.abs(self.megaNodes - 1)
        connectivityTest = connectivityTest.astype(int)
        # for i in range(self.xNodes):
        # 	for j in range(self.yNodes):
        # 		connectivityTest[i][j] = int(abs(self.megaNodes[i][j][2] - 1))

        G2G.compactLabeling(connectivityTest, self.yNodes, self.xNodes, True)
        if G2G.getMaxLabel() > 1:
            optimizationIndex -= .5

        return optimizationIndex

    def marginNormSSI(self):
        SSI = abs(abs(self.xBoxMax - self.xMax) - abs(self.xMin - self.xBoxMin)) / (2 * abs(self.xBoxMax - self.xBoxMin)) + abs(
            abs(self.yBoxMax - self.yMax) - abs(self.yMin - self.yBoxMin)) / (2 * abs(self.yBoxMax - self.yBoxMin))

        return SSI

    def getPolygonArea(self):
        area = 0
        numPoints = len(self.polygonCoordinates)
        j = numPoints - 1  # The last vertex is the 'previous' one to the first
        for i in range(numPoints):
            area = area + abs((self.polygonCoordinates[j][0] + self.polygonCoordinates[i][0]) * (
                    self.polygonCoordinates[j][1] - self.polygonCoordinates[i][1]))
            j = i  # j is previous vertex to i

        return area / 2

    def getBoundingBoxArea(self):
        return self.xRngMeters * self.yRngMeters

    def getMegaNodesInCount(self):
        return self.megaNodesCount

    def getMegaNodes(self):
        megaNodesAux = np.zeros((self.xNodes, self.yNodes, 3))
        for i in range(self.xNodes):
            for j in range(self.yNodes):
                megaNodesAux[i][j][0] = self.xMin + i * self.nodeDistance + self.shiftX
                megaNodesAux[i][j][1] = self.yMin + j * self.nodeDistance + self.shiftY
                megaNodesAux[i][j][2] = self.megaNodes[i][j]
        return megaNodesAux

    def getSubNodes(self):
        subNodes = np.zeros((2 * self.xNodes, 2 * self.yNodes, 3))
        for i in range(self.xNodes):
            aux_0 = self.xMin + i * self.nodeDistance + self.shiftX
            for j in range(self.yNodes):
                aux_1 = self.yMin + j * self.nodeDistance + self.shiftY
                subNodes[2 * i][2 * j + 1][0] = aux_0 - self.nodeIntervalOffset
                subNodes[2 * i][2 * j + 1][1] = aux_1 + self.nodeIntervalOffset

                subNodes[2 * i + 1][2 * j + 1][0] = aux_0 + self.nodeIntervalOffset
                subNodes[2 * i + 1][2 * j + 1][1] = aux_1 + self.nodeIntervalOffset

                subNodes[2 * i][2 * j][0] = aux_0 - self.nodeIntervalOffset
                subNodes[2 * i][2 * j][1] = aux_1 - self.nodeIntervalOffset

                subNodes[2 * i + 1][2 * j][0] = aux_0 + self.nodeIntervalOffset
                subNodes[2 * i + 1][2 * j][1] = aux_1 - self.nodeIntervalOffset

                subNodes[2 * i][2 * j + 1][2] = self.megaNodes[i][j]
                subNodes[2 * i + 1][2 * j + 1][2] = self.megaNodes[i][j]
                subNodes[2 * i][2 * j][2] = self.megaNodes[i][j]
                subNodes[2 * i + 1][2 * j][2] = self.megaNodes[i][j]

        return subNodes
