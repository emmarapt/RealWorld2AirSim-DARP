# from com.app.choosepath.pathPlanning.handleGeo.coordinates import *
from RealWorld.handleGeo.coordinates import WGS84
from RealWorld.handleGeo.coordinates.WGS84 import WGS84_class
from RealWorld.handleGeo.coordinates.NED import NED
from RealWorld.handleGeo.coordinates.ECEF import ECEF

import math
import numpy as np


class ConvCoords(object):

    def __init__(self, geoCoords, geoObstacles):

        # self.reference = None
        # self.polygonWGS84 = [[]]
        # self.polygonNED = [[]]
        # self.obstaclesWGS84 = [[[]]]
        # self.obstaclesNED = [[[]]]
        # self.waypointsWGS84 = None
        # self.waypointsNED = None
        self.geoCoords = geoCoords
        self.geoObstacles = geoObstacles

        self.obstaclesWGS84 = geoObstacles
        self.obstaclesNED = [[] for _ in range(len(self.geoObstacles))]

        self.reference = WGS84_class(math.radians(self.geoCoords[0][0]), math.radians(self.geoCoords[0][1]), 0)
        #self.reference = WGS84_class(0, 0, 0)

    def polygonWGS84ToNED(self):
        self.polygonWGS84 = self.geoCoords
        self.reference = WGS84_class(math.radians(self.geoCoords[0][0]), math.radians(self.geoCoords[0][1]), 0)

        self.polygonNED = self.WGS84toNED(self.geoCoords)

        return self.polygonNED

    def convWGS84ToNED(self, geoCoords):

        # cartCoords = np.array([len(geoCoords)][geoCoords[0].Length])
        cartCoords = self.WGS84toNED(geoCoords)

        return cartCoords

    def obstaclesToNED(self):


        for i in range(len(self.geoObstacles)):
            self.obstaclesNED[i] = self.WGS84toNED(self.geoObstacles[i])

        return self.obstaclesNED

    def WGS84toNED(self, geoCoords):


        cartCoords = np.zeros((len(geoCoords), len(geoCoords[0])))

        for i in range(len(geoCoords)):
            wgs84 = WGS84_class(math.radians(geoCoords[i][0]), math.radians(geoCoords[i][1]), 0)

            ned = WGS84.displacement(self.reference, wgs84)

            cartCoords[i][0] = ned.north
            cartCoords[i][1] = ned.east

        return cartCoords

    def NEDToWGS84(self, cartCoords):
        # wgs84 = None
        # ned = None
        local = []

        self.waypointsNED = cartCoords
        j = 0
        while j < len(cartCoords):
            geoCoords = []
            i = 0
            while i < len(cartCoords[j]):
                ned = NED(cartCoords[j][i][0], cartCoords[j][i][1], 0)
                wgs84 = WGS84.displace(self.reference, ned)
                geoCoords.append([math.degrees(wgs84.latitude), math.degrees(wgs84.longitude)])
                i += 1
            local.append(geoCoords)
            j += 1
        self.waypointsWGS84 = local

        return self.waypointsWGS84

    def getObstaclesWGS84(self):
        return self.obstaclesWGS84

    def getObstaclesNED(self):
        return self.obstaclesNED

    def getPolygonWGS84(self):
        return self.polygonWGS84

    def getPolygonNED(self):
        return self.polygonNED

    def getWaypointsWGS84(self):
        return self.waypointsWGS84

    def getWaypointsNED(self):
        return self.waypointsNED
