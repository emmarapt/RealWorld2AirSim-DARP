import json
import math
import time
import numpy as np
from RealWorld.handleGeo.ConvCoords import ConvCoords
from RealWorld.handleGeo.NodesInPoly import NodesInPoly
from RealWorld.nodesPlacementOptimization.SimulatedAnnealing import SimulatedAnnealing
from RealWorld.nodesPlacementOptimization.Rotate import Rotate
from RealWorld.ConnectComponent import ConnectComponent
from RealWorld.visualizeNEDPaths.visualizeNEDPaths import PlotNEDPaths
from RealWorld.handleGeo.GridPathsToNED import GridPathsToNED
from RealWorld.handleGeo import Dist


class real_world:
    def __init__(self):
        """ DEFINE THE DRONE'S SPECS """
        # # Phantom 4 pro Image Sensor specs
        self.HFOV = 73.4
        self.hRes = 5472
        self.ImageWidth = 5472
        self.ImageHeight = 3648
        self.SensorWidth = 13.2
        self.SensorHeight = 8
        self.FocalLength = 8.8

        self.real_world_parameters()

    def real_world_parameters(self):
        file = open('inputVariables_test.json')
        data = json.load(file)

        self.geoCoords = []
        self.geoObstacles = []

        for i in data['polygon']:
            self.geoCoords.append([i.get("lat"), i.get("long")])

        if len(data['obstacles']) > 0:
            self.geoObstacles = [[] for _ in range(len(data['obstacles']))]

            for i in range(len(data['obstacles'])):
                for j in data['obstacles'][i]:
                    self.geoObstacles[i].append([j.get("lat"), j.get("long")])

        self.altitude = data['altitude']
        self.sidelap = data['sidelap']

        self.scanDist = float("{:.2f}".format(self.covered() * (1 - self.sidelap / 100)))
        print("Scanning Distance:", self.scanDist)

        self.droneNo = data['droneNo']
        self.portions = data['rPortions']
        self.pathsStrictlyInPoly = data['pathsStrictlyInPoly']
        self.optimal_init_pos = data['OptimalInitPos']

        self.randomInitPos = True  # If false define in WGS84 the initialPos of the drones
        self.notEqualPortions = True
        self.initial_positions = []

        for i in data['initialPos']:
            self.initial_positions.append([i.get("lat"), i.get("long")])

    def geo2cart(self):
        # Convert geographical to local cartesian coordinates
        self.NED_Coords = ConvCoords(self.geoCoords, self.geoObstacles).polygonWGS84ToNED()

        if len(self.geoObstacles) > 0:
            self.obstNED = ConvCoords(self.geoCoords, self.geoObstacles).obstaclesToNED()
        else:
            self.obstNED = []  # [np.complex64(x) for x in range(0)]

        # Rotation and shift optimization (SimulatedAnnealing)
        start = time.time()
        optimalParameters = SimulatedAnnealing()
        optimalParameters.run(self.NED_Coords, self.obstNED, self.scanDist)

        self.rotate = Rotate()
        self.theta = optimalParameters.getOptimalTheta()
        self.shiftX = optimalParameters.getOptimalShiftX()
        self.shiftY = optimalParameters.getOptimalShiftY()
        self.rotate.setTheta(self.theta)
        cart = self.rotate.rotatePolygon(self.NED_Coords)

        cartObst = [[] for _ in range(len(self.obstNED))]

        for i in range(len(self.obstNED)):
            cartObst[i] = self.rotate.rotatePolygon(self.obstNED[i])

        print(f"Time needed to find the optimal solution: {time.time() - start} seconds")
        print(f"- Optimal theta: {self.theta}")
        print(f"- Optimal shift in X axis: {self.shiftX}")
        print(f"- Optimal shift in Y axis: {self.shiftY}")

        # Build grid for paths
        NodesInPoly_var = NodesInPoly(cart, cartObst, scanDist=self.scanDist,
                                      pathsStrictlyInPoly=self.pathsStrictlyInPoly,
                                      hideInfo=False, shiftX=self.shiftX, shiftY=self.shiftY)

        megaNodesIn = NodesInPoly_var.getMegaNodesInCount()
        self.megaNodes = NodesInPoly_var.getMegaNodes()
        self.subNodes = NodesInPoly_var.getSubNodes()

        print(f"User's defined polygon area: {NodesInPoly_var.getPolygonArea()} square meters")

        # Cases that cannot run
        if megaNodesIn < 1:
            print(
                "!!! The defined area is too small to fit path for the asked scanning density! Try again a larger area or a smaller scanning density !!!")
        elif megaNodesIn < self.droneNo:
            print("!!! Not enough space to have at least minimum length paths for every drone !!!")
            print(f"With this configuration you can deploy {megaNodesIn} drones at most")
            print(f"Number of drones automatically changed to {megaNodesIn} in order to have a solution")
            print(
                "If you are not satisfied with the solution you could try to rerun for:\n - Larger area\n - Smaller scanning distance")

            self.droneNo = megaNodesIn

    def get_DARP_params(self):
        # DARP parameters
        rows = len(self.megaNodes)
        cols = len(self.megaNodes[0])

        """ In DARPgrid 0 stands for free space 
            1 stands for Obstacle
            2 stands for Drone
        """
        DARPgrid = self.megaNodes[:, :, 2].astype(int)

        # """ ADDON for DARP_initial_positions module """
        grid_positions = np.arange(0, rows * cols).reshape(rows, cols)
        obs_pos_to_grid = np.where(DARPgrid == 1)
        obs_pos_to_grid = np.asarray(obs_pos_to_grid).T
        obs_pos = []

        for elem in obs_pos_to_grid:
            obs_pos.append(grid_positions[elem[0], elem[1]])

        return rows, cols, obs_pos

    def GSD(self):
        return ((2 * self.altitude * self.getTanFromDegrees(self.HFOV / 2)) / self.hRes) * 100

    def getTanFromDegrees(self, degrees):
        return math.tan(degrees * math.pi / 180)

    def covered(self):
        return 2 * self.altitude * self.getTanFromDegrees(self.HFOV / 2)

    def GSDh(self):
        return ((self.altitude * 100) * (self.SensorHeight / 10)) / ((self.FocalLength / 10) * self.ImageHeight)

    def GSDw(self):
        return ((self.altitude * 100) * (self.SensorWidth / 10)) / ((self.FocalLength / 10) * self.ImageWidth)
