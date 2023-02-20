"""For Python 3.9"""

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
        if len(cartObst) == 0:
            self.cartObst = np.zeros((1, 4, 2))
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
    # @TimeItCount
    def strictlyInPolyWrap(xMin, yMin, nodeDistance, hideInfo, nodeIntervalOffset, shiftX, shiftY, polygonCoordinates,
                           xNodes, yNodes, cartObst, initialization, computation):

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
        SSI = abs(abs(self.xBoxMax - self.xMax) - abs(self.xMin - self.xBoxMin)) / (
                    2 * abs(self.xBoxMax - self.xBoxMin)) + abs(
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


"""For Python 3.6"""

# from RealWorld.helpers import MinMax
# import numpy as np
# from RealWorld.handleGeo import InPolygon
# from RealWorld.ConnectComponent import ConnectComponent
# import math
# from numba import njit
#
#
# class NodesInPoly(object):
#
#     def __init__(self, cartCoords, cartObst, scanDist, pathsStrictlyInPoly, hideInfo, shiftX, shiftY):
#
#         self.scanDist = scanDist
#         self.pathsStrictlyInPoly = pathsStrictlyInPoly
#         self.hideInfo = hideInfo
#
#         self.cartObst = np.array(cartObst)
#         # if len(cartObst) ==0:
#         #     self.cartObst = [np.complex64(x) for x in range(0)]
#         # else:
#         #self.cartObst = cartObst
#
#         self.shiftX = shiftX
#         self.shiftY = shiftY
#
#         self.nodeDistance = 2 * self.scanDist
#         self.nodeIntervalOffset = self.scanDist / 2
#         self.polygonCoordinates = cartCoords
#
#
#         self.xMax = MinMax.xMax(self.polygonCoordinates) + self.nodeDistance
#         self.xMin = MinMax.xMin(self.polygonCoordinates) - self.nodeDistance
#         self.yMax = MinMax.yMax(self.polygonCoordinates) + self.nodeDistance
#         self.yMin = MinMax.yMin(self.polygonCoordinates) - self.nodeDistance
#
#         self.xRngMeters = self.xMax - self.xMin
#         self.yRngMeters = self.yMax - self.yMin
#
#         self.xNodes = int((self.xRngMeters / self.nodeDistance))
#         self.yNodes = int((self.yRngMeters / self.nodeDistance))
#
#         # self.megaNodes = np.zeros((self.xNodes, self.yNodes, 3))
#         # self.subNodes = np.zeros((2 * self.xNodes, 2 * self.yNodes, 3))
#
#         if not hideInfo:
#             print("Bounding box: ", self.xRngMeters, " x ", self.yRngMeters, " meters")
#             print("xNodes: ", self.xNodes)
#             print("yNodes: ", self.yNodes)
#             print("Total number of mega-nodes: ", self.xNodes * self.yNodes)
#
#         xInter = self.xMax - self.xMin - (self.xNodes - 1) * self.nodeDistance
#         yInter = self.yMax - self.yMin - (self.yNodes - 1) * self.nodeDistance
#
#         if self.pathsStrictlyInPoly:
#             self.subNodes, self.megaNodes, self.megaNodesCount = self.strictlyInPoly(self.xMin, self.yMin,
#                                                                                      self.nodeDistance, self.hideInfo,
#                                                                                      self.nodeIntervalOffset,
#                                                                                      self.shiftX, self.shiftY,
#                                                                                      self.polygonCoordinates,
#                                                                                      self.checkInPolyAndObstacle,
#                                                                                      self.xNodes, self.yNodes,
#                                                                                      self.cartObst)
#         else:
#             self.betterCoverage()
#
#     @staticmethod
#     @njit()
#     def strictlyInPoly(xMin, yMin, nodeDistance, hideInfo, nodeIntervalOffset, shiftX, shiftY, polygonCoordinates,
#                        checkInPolyAndObstacle, xNodes, yNodes, cartObst):
#
#         megaNodesCount = 0
#         megaNodes = np.zeros((xNodes, yNodes, 3))
#         for i in range(xNodes):
#             for j in range(yNodes):
#                 megaNodes[i][j][0] = xMin + i * nodeDistance + shiftX
#                 megaNodes[i][j][1] = yMin + j * nodeDistance + shiftY
#
#         subNodes = np.zeros((2 * xNodes, 2 * yNodes, 3))
#         for i in range(xNodes):
#             for j in range(yNodes):
#
#                 subNodes[2 * i][2 * j + 1][0] = megaNodes[i][j][0] - nodeIntervalOffset
#                 subNodes[2 * i][2 * j + 1][1] = megaNodes[i][j][1] + nodeIntervalOffset
#
#                 subNodes[2 * i + 1][2 * j + 1][0] = megaNodes[i][j][0] + nodeIntervalOffset
#                 subNodes[2 * i + 1][2 * j + 1][1] = megaNodes[i][j][1] + nodeIntervalOffset
#
#                 subNodes[2 * i][2 * j][0] = megaNodes[i][j][0] - nodeIntervalOffset
#                 subNodes[2 * i][2 * j][1] = megaNodes[i][j][1] - nodeIntervalOffset
#
#                 subNodes[2 * i + 1][2 * j][0] = megaNodes[i][j][0] + nodeIntervalOffset
#                 subNodes[2 * i + 1][2 * j][1] = megaNodes[i][j][1] - nodeIntervalOffset
#                 if InPolygon.check([subNodes[2 * i][2 * j + 1][0], subNodes[2 * i][2 * j + 1][1]],
#                                    polygonCoordinates) and InPolygon.check(
#                     [subNodes[2 * i + 1][2 * j + 1][0], subNodes[2 * i + 1][2 * j + 1][1]],
#                     polygonCoordinates) and InPolygon.check(
#                     [subNodes[2 * i][2 * j][0], subNodes[2 * i][2 * j][1]],
#                     polygonCoordinates) and InPolygon.check(
#                     [subNodes[2 * i + 1][2 * j][0], subNodes[2 * i + 1][2 * j][1]],
#                     polygonCoordinates):
#
#
#                     subNodes, megaNodes, megaNodesCount = checkInPolyAndObstacle(i, j, subNodes, megaNodes, cartObst,
#                                                                                  megaNodesCount)
#
#                 else:
#                     megaNodes[i][j][2] = 1 # Obstacle
#
#                 subNodes[2 * i][2 * j + 1][2] = megaNodes[i][j][2]
#                 subNodes[2 * i + 1][2 * j + 1][2] = megaNodes[i][j][2]
#                 subNodes[2 * i][2 * j][2] = megaNodes[i][j][2]
#                 subNodes[2 * i + 1][2 * j][2] = megaNodes[i][j][2]
#
#             #                // Uncomment to print all the sub-nodes
#             #                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i][2*j][1]+" ---> "+subNodes[2*i][2*j][2])
#             #                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1]+" ---> "+subNodes[2*i+1][2*j][2])
#             #                System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1]+" ---> "+subNodes[2*i][2*j+1][2])
#             #                System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1]+" ---> "+subNodes[2*i+1][2*j+1][2])
#             #
#             #                // Uncomment to print the sub-nodes that will be used for trajectories
#             #                if (megaNodes[i][j][2]!=1){
#             #                    System.out.println(subNodes[2*i][2*j][0]+", "+subNodes[2*i][2*j][1])
#             #                    System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1])
#             #                    System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1])
#             #                    System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1])
#             #                }
#
#         if not hideInfo:
#             print("Number of mega-nodes that are used for STC: ", megaNodesCount)
#             print("Number of sub-nodes that will be used for trajectories: ", 4.0 * megaNodesCount)
#         return subNodes, megaNodes, megaNodesCount
#
#     def betterCoverage(self, xMin, yMin, nodeDistance, hideInfo):
#
#         self.megaNodesCount = 0
#         self.megaNodes = np.zeros((self.xNodes, self.yNodes, 3))
#         for i in range(self.xNodes):
#             for j in range(self.yNodes):
#                 self.megaNodes[i][j][0] = xMin + i * nodeDistance + self.shiftX
#                 self.megaNodes[i][j][1] = yMin + j * nodeDistance + self.shiftY
#
#                 if InPolygon.check([self.megaNodes[i][j][0], self.megaNodes[i][j][1]], self.polygonCoordinates):
#                     self.megaNodes[i][j][2] = 0
#                     self.megaNodesCount += 1
#                 else:
#                     self.megaNodes[i][j][2] = 1
#
#         if not hideInfo:
#             print("Number of mega-nodes inside polygon: " + self.megaNodesCount)
#             print("Number of sub-nodes that will be used for trajectories: " + 4.0 * self.megaNodesCount)
#
#         self.subNodes = np.zeros((2 * self.xNodes, 2 * self.yNodes, 3))
#
#         for i in range(self.xNodes):
#             for j in range(self.yNodes):
#                 self.subNodes[2 * i][2 * j + 1][0] = self.megaNodes[i][j][0] - self.nodeIntervalOffset
#                 self.subNodes[2 * i][2 * j + 1][1] = self.megaNodes[i][j][1] + self.nodeIntervalOffset
#
#                 self.subNodes[2 * i + 1][2 * j + 1][0] = self.megaNodes[i][j][0] + self.nodeIntervalOffset
#                 self.subNodes[2 * i + 1][2 * j + 1][1] = self.megaNodes[i][j][1] + self.nodeIntervalOffset
#
#                 self.subNodes[2 * i][2 * j][0] = self.megaNodes[i][j][0] - self.nodeIntervalOffset
#                 self.subNodes[2 * i][2 * j][1] = self.megaNodes[i][j][1] - self.nodeIntervalOffset
#
#                 self.subNodes[2 * i + 1][2 * j][0] = self.megaNodes[i][j][0] + self.nodeIntervalOffset
#                 self.subNodes[2 * i + 1][2 * j][1] = self.megaNodes[i][j][1] - self.nodeIntervalOffset
#
#                 self.checkInPolyAndObstacle(i, j)
#
#                 self.subNodes[2 * i][2 * j + 1][2] = self.megaNodes[i][j][2]
#                 self.subNodes[2 * i + 1][2 * j + 1][2] = self.megaNodes[i][j][2]
#                 self.subNodes[2 * i][2 * j][2] = self.megaNodes[i][j][2]
#                 self.subNodes[2 * i + 1][2 * j][2] = self.megaNodes[i][j][2]
#
#             #                // Uncomment to print all the sub-nodes
#             #                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i][2*j][1]+" ---> "+subNodes[2*i][2*j][2])
#             #                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1]+" ---> "+subNodes[2*i+1][2*j][2])
#             #                System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1]+" ---> "+subNodes[2*i][2*j+1][2])
#             #                System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1]+" ---> "+subNodes[2*i+1][2*j+1][2])
#             #
#             #                // Uncomment to print the sub-nodes that will be used for trajectories
#             #                if (nodes[i][j][2]!=1){
#             #                    System.out.println(subNodes[2*i][2*j][0]+", "+subNodes[2*i][2*j][1])
#             #                    System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1])
#             #                    System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1])
#             #                    System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1])
#             #                }
#
#     @staticmethod
#     @njit()
#     def checkInPolyAndObstacle(i, j, subNodes, megaNodes, cartObst, megaNodesCount):
#         # if len(cartObst) > 0:
#         #     for k in range(len(cartObst)):
#         #         if InPolygon.check([subNodes[2 * i][2 * j + 1][0], subNodes[2 * i][2 * j + 1][1]],
#         #                            cartObst[k]) or InPolygon.check(
#         #             [subNodes[2 * i + 1][2 * j + 1][0], subNodes[2 * i + 1][2 * j + 1][1]],
#         #             cartObst[k]) or InPolygon.check(
#         #             [subNodes[2 * i][2 * j][0], subNodes[2 * i][2 * j][1]],
#         #             cartObst[k]) or InPolygon.check(
#         #             [subNodes[2 * i + 1][2 * j][0], subNodes[2 * i + 1][2 * j][1]], cartObst[k]):
#         #             megaNodes[i][j][2] = 1
#         #         elif megaNodes[i][j][2] != 1:
#         #             megaNodes[i][j][2] = 0
#         #             megaNodesCount += 1
#
#         if megaNodes[i][j][2] != 1:
#             megaNodes[i][j][2] = 0  # free space
#             megaNodesCount += 1
#         return subNodes, megaNodes, megaNodesCount
#
#     def getOptimizationIndex(self):
#
#         a = 0.68
#         b = 0.32
#         c = 0.25
#
#         polygonArea = self.getPolygonArea()
#
#         nodesInTerm = (self.megaNodesCount * math.pow(self.nodeDistance, 2)) / polygonArea
#         minBBAreaTerm = polygonArea / self.getBoundingBoxArea()
#
#         if self.megaNodesCount > 0:
#             equalMarginsTerm = self.marginNormSSI()
#
#         else:
#             equalMarginsTerm = 1
#
#         optimizationIndex = a * nodesInTerm + b * minBBAreaTerm - c * equalMarginsTerm
#
#         """ WITHOUT ConnectComponent"""
#         G2G = ConnectComponent()
#
#         connectivityTest = np.zeros((self.xNodes, self.yNodes))
#         connectivityTest[:] = np.abs(self.megaNodes[:, :, 2] - 1)
#         connectivityTest = connectivityTest.astype(int)
#         # for i in range(self.xNodes):
#         # 	for j in range(self.yNodes):
#         # 		connectivityTest[i][j] = int(abs(self.megaNodes[i][j][2] - 1))
#
#         G2G.compactLabeling(connectivityTest, self.yNodes, self.xNodes, True)
#         if G2G.getMaxLabel() > 1:
#             optimizationIndex -= .5
#
#         return optimizationIndex
#
#     def marginNormSSI(self):
#         coords = np.zeros((4 * self.megaNodesCount, 2))
#
#         # c = 0
#         # for i in range(2 * self.xNodes):
#         # 	for j in range(2 * self.yNodes):
#         # 		if self.subNodes[i][j][2] != 1:
#         # 			coords[c][0] = self.subNodes[i][j][0]
#         # 			coords[c][1] = self.subNodes[i][j][1]
#         # 			c += 1
#
#         inds = np.where(self.subNodes[:, :, 2].ravel() != 1)
#         a = self.subNodes[:, :, 0].ravel()[inds]
#         b = self.subNodes[:, :, 1].ravel()[inds]
#         coords = np.column_stack((a, b))
#         # print((coords==coords).all())
#         xBoxMax = MinMax.xMax(coords)
#         xBoxMin = MinMax.xMin(coords)
#         yBoxMax = MinMax.yMax(coords)
#         yBoxMin = MinMax.yMin(coords)
#
#         SSI = abs(abs(xBoxMax - self.xMax) - abs(self.xMin - xBoxMin)) / (2 * abs(xBoxMax - xBoxMin)) + abs(
#             abs(yBoxMax - self.yMax) - abs(self.yMin - yBoxMin)) / (2 * abs(yBoxMax - yBoxMin))
#
#         return SSI
#
#     def getPolygonArea(self):
#         area = 0
#         numPoints = len(self.polygonCoordinates)
#         j = numPoints - 1  # The last vertex is the 'previous' one to the first
#         for i in range(numPoints):
#             area = area + abs((self.polygonCoordinates[j][0] + self.polygonCoordinates[i][0]) * (
#                     self.polygonCoordinates[j][1] - self.polygonCoordinates[i][1]))
#             j = i  # j is previous vertex to i
#
#         return area / 2
#
#     def getBoundingBoxArea(self):
#         return self.xRngMeters * self.yRngMeters
#
#     def getMegaNodesInCount(self):
#         return self.megaNodesCount
#
#     def getMegaNodes(self):
#         return self.megaNodes
#
#     def getSubNodes(self):
#         return self.subNodes
#
