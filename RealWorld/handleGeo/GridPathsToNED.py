import sys
sys.path.append('DARP_Optimal_Initial_Positions/DARP')
from Visualization import visualize_paths

class GridPathsToNED:

    def __init__(self, FinalPaths, initial_positions, droneNo, subNodes, rotate):
        self.FinalPaths = FinalPaths
        self.initial_positions = initial_positions
        self.droneNo = droneNo
        self.subNodes = subNodes
        self.rotate = rotate
        self.missionWaypointsNED = [[] for _ in range(self.droneNo)]

    def init_posGRIDToNED(self):
        init_pos_NED = []

        for k in range(self.droneNo):
            xInit = 2 * self.initial_positions[k][0]
            yInit = 2 * self.initial_positions[k][1]
            init_pos_NED.append([self.subNodes[xInit][yInit][0], self.subNodes[xInit][yInit][1]])
            initial_positions_NED = self.rotate.rotateBackWaypoints(init_pos_NED)
        return initial_positions_NED

    def getWaypointsNED(self):
        for drone in range(self.droneNo):
            iWaypoints = []
            for j in range(len(self.FinalPaths[drone])):
                x = self.FinalPaths[drone][j][0]
                y = self.FinalPaths[drone][j][1]
                iWaypoints.append([self.subNodes[x][y][0], self.subNodes[x][y][1]]) 

            x = self.FinalPaths[drone][0][0]
            y = self.FinalPaths[drone][0][1]
            
            iWaypoints.append([self.subNodes[x][y][0], self.subNodes[x][y][1]])
            WP = self.rotate.rotateBackWaypoints(iWaypoints)
            self.missionWaypointsNED[drone] = WP 

        return self.missionWaypointsNED