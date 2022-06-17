import sys
sys.path.append('DARP_Optimal_Initial_Positions/DARP')


class GridPathsToNED:

    def __init__(self, optimal_init_pos, object, droneNo, subNodes, rotate):
        self.optimal_init_pos = optimal_init_pos
        self.object = object
        self.droneNo = droneNo
        self.subNodes = subNodes
        self.rotate = rotate
        self.missionWaypointsNED = [[] for _ in range(self.droneNo)]

    def init_posGRIDToNED(self):
        # init_pos = self.object.best_trial.darp_instance.initial_positions
        init_pos_NED = []
        for k in range(self.droneNo):

            if self.optimal_init_pos:
                xInit = 2 * self.object.best_trial.darp_instance.initial_positions[k][0]
                yInit = 2 * self.object.best_trial.darp_instance.initial_positions[k][1]
            else:
                xInit = 2 * self.object.darp_instance.initial_positions[k][0]
                yInit = 2 * self.object.darp_instance.initial_positions[k][1]
            
            init_pos_NED.append([self.subNodes[xInit][yInit][0], self.subNodes[xInit][yInit][1]])

            initial_positions_NED = self.rotate.rotateBackWaypoints(init_pos_NED)
        return initial_positions_NED

    def getWaypointsNED(self):

        if self.optimal_init_pos:
            TypesOfLines = self.object.best_trial.TypesOfLines
        else:
            TypesOfLines = self.object.TypesOfLines

        # // Clockwise path
        for i in range(len(TypesOfLines)):
            for j in range(len(TypesOfLines[0])):
                self.subNodes[i][j][2] = TypesOfLines[i][j][0]

        # // Calculate paths for all drones, for all modes (see below) and keep the paths with the minimum turns
        allDirectionsWaypoints = [[[] for _ in range(4)] for _ in range(self.droneNo)]

        for mode in range(4):
            for k in range(self.droneNo):
                iWaypoints = []

                if self.optimal_init_pos:
                    xInit = 2 * self.object.best_trial.darp_instance.initial_positions[k][0]
                    yInit = 2 * self.object.best_trial.darp_instance.initial_positions[k][1]
                else:
                    xInit = 2 * self.object.darp_instance.initial_positions[k][0]
                    yInit = 2 * self.object.darp_instance.initial_positions[k][1]

                i = xInit
                j = yInit

                iWaypoints.append([self.subNodes[xInit][yInit][0], self.subNodes[xInit][yInit][1]])
                # iWaypoints = [
                #     [self.subNodes[[i for i in range(len(self.subNodes))]][[j for j in range(len(self.subNodes[0]))][0]][0],
                #      self.subNodes[[i for i in range(len(self.subNodes))]][[j for j in range(len(self.subNodes[0][0]))][0]][1]]]

                condition = True
                while condition:

                    prevState = self.subNodes[i][j][2]

                    if self.subNodes[i][j][2] == 1.0:
                        i -= 1
                    elif self.subNodes[i][j][2] == 2.0:
                        j -= 1
                    elif self.subNodes[i][j][2] == 3.0:
                        j += 1
                    elif self.subNodes[i][j][2] == 4.0:
                        i += 1

                    if prevState != self.subNodes[i][j][2] or (
                            self.subNodes[i][j][0] == iWaypoints[0][0] and self.subNodes[i][j][1] == iWaypoints[0][1]):
                        iWaypoints.append([self.subNodes[i][j][0], self.subNodes[i][j][1]])

                    condition = not (i == xInit and j == yInit)
                WP = self.rotate.rotateBackWaypoints(iWaypoints)

                allDirectionsWaypoints[k][mode].append(WP)

        # // Keep orientation with less turns
        for i in range(self.droneNo):
            ind = 0
            min = len(allDirectionsWaypoints[i][0][0])
            for j in range(1, 4):
                if len(allDirectionsWaypoints[i][j][0]) < min:
                    min = len(allDirectionsWaypoints[i][j][0])
                    ind = j
            self.missionWaypointsNED[i].append(allDirectionsWaypoints[i][ind][0])

        return self.missionWaypointsNED