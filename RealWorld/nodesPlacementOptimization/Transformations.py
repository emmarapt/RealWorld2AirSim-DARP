from RealWorld.handleGeo.NodesInPoly import NodesInPoly
from RealWorld.nodesPlacementOptimization.Rotate import Rotate


class Transformations(object):

    # def __init__(self):
    #
    #     self.theta = 0
    #     self.shiftX = 0
    #     self.shiftY = 0

    def getTheta(self):
        return self.theta

    def getShiftX(self):
        return self.shiftX

    def getShiftY(self):
        return self.shiftY

    def setTheta(self, theta):
        self.theta = theta

    def setShiftX(self, shiftX):
        self.shiftX = shiftX

    def setShiftY(self, shiftY):
        self.shiftY = shiftY

    def rotateAndShift(self, cart, cartObst, scanDist):

        rotate = Rotate()
        rotate.setTheta(self.theta)
        rotated = rotate.rotatePolygon(cart)
        rotatedObst = []


        if len(cartObst) > 0:
            rotatedObst = [[] for _ in range(len(cartObst))]
            for i in range(len(cartObst)):
                rotatedObst[i] = rotate.rotatePolygon(cartObst[i])

        testgrid = NodesInPoly(rotated, rotatedObst, scanDist=scanDist,
                              pathsStrictlyInPoly=True,
                              hideInfo=True, shiftX=self.shiftX, shiftY=self.shiftY)
        optimizationIndex = testgrid.getOptimizationIndex()  # NodesInPoly(rotated, rotatedObst, scanDist, True, True, self.shiftX, self.shiftY).getOptimizationIndex()

        return optimizationIndex

    def shift(self, cart, cartObst, scanDist):

        optimize = NodesInPoly(cart, cartObst, scanDist, True, True, self.shiftX, self.shiftY)
        megaNodesInCount = optimize.getMegaNodesInCount()

        return megaNodesInCount
