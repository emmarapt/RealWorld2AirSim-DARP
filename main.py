import time
import sys
import argparse
import random
import numpy as np
import cProfile
from SimWorld.airlearning_runner import run_airlearning
from SimWorld.airsim_runner import run_airsim
from RealWorld.real_world_parameter_parser import real_world
from RealWorld.handleGeo.GridPathsToNED import GridPathsToNED
from RealWorld.handleGeo import Dist
from RealWorld.visualizeNEDPaths.visualizeNEDPaths import PlotNEDPaths
from RealWorld.handleGeo.GridPathsToNED import GridPathsToNED
from RealWorld.handleGeo import Dist

sys.path.append('DARP_Optimal_Initial_Positions')
from darp_x_optuna import optimize
from DARP.multiRobotPathPlanner import MultiRobotPathPlanner


def run_experiment(optimal_init_pos, number_of_trials, AirSim, AirLearning):
    real_world_parameters = real_world()
    real_world_parameters.geo2cart()
    rows, cols, obstacles_positions = real_world_parameters.get_DARP_params()

    if optimal_init_pos:  # Runs with OPTUNA !!
        optimization = optimize(rows, cols, number_of_trials, False, [], obstacles_positions, False,
                                real_world_parameters.droneNo)

        optimization.optimize()

        FinalPaths = optimization.best_trial.best_case.paths
        initial_positions = optimization.best_trial.darp_instance.initial_positions


        NEDdata = GridPathsToNED(FinalPaths, initial_positions, real_world_parameters.droneNo,
                                 real_world_parameters.subNodes, real_world_parameters.rotate)

        #NEDdata = GridPathsToNED(optimal_init_pos, optimization, real_world_parameters.droneNo, real_world_parameters.subNodes, real_world_parameters.rotate)
        init_posNED = NEDdata.init_posGRIDToNED()

        # WaypointsNED are in the form of WaypointsNED[DroneNo][0]
        WaypointsNED = NEDdata.getWaypointsNED()

    else:  # Runs with random init pos !!

        # Put drones in initial positions (close to physical or random)
        randomInitPos = True

        count = 0
        while True:
            print("\n!!! DARP will run for random initial positions !!! ")
            grid = np.arange(0, rows * cols).reshape(rows, cols)

            DARPgrid = initializeDARPGrid(randomInitPos, rows, cols, real_world_parameters.initial_positions,
                                          real_world_parameters.megaNodes,
                                          real_world_parameters.theta, real_world_parameters.shiftX,
                                          real_world_parameters.shiftY,
                                          real_world_parameters.droneNo)

            positions_DARPgrid = np.where(DARPgrid == 2)

            positions_DARPgrid = np.asarray(positions_DARPgrid).T
            positions = []

            for elem in positions_DARPgrid:
                positions.append(grid[elem[0], elem[1]])

            poly = MultiRobotPathPlanner(rows, cols, real_world_parameters.notEqualPortions, positions,
                                         real_world_parameters.portions, obstacles_positions, False)
            count += 1

            if poly.DARP_success or count > 5:
                break

            print("\n!!! DARP will rerun for random initial positions !!! ")

        if count == 5:
            print("DARP did not manage to find a solution for the given configuration!")
            print("Try to alter one or more of:")
            print("- Scanning distance")
            print("- Number of drones")
            print("Given area")
            exit()
        else:


            FinalPaths = poly.best_case.paths
            initial_positions = poly.darp_instance.initial_positions

            NEDdata = GridPathsToNED(FinalPaths, initial_positions, real_world_parameters.droneNo,
                                     real_world_parameters.subNodes, real_world_parameters.rotate)
            # NEDdata = GridPathsToNED(optimal_init_pos, poly, real_world_parameters.droneNo,
            #                          real_world_parameters.subNodes, real_world_parameters.rotate)

            init_posNED = NEDdata.init_posGRIDToNED()
            WaypointsNED = NEDdata.getWaypointsNED()

    # """ Visualize NED Paths """
    PlotNEDPaths(real_world_parameters.NED_Coords, real_world_parameters.obstNED, real_world_parameters.droneNo,
                 WaypointsNED, init_posNED, optimal_init_pos).plot()

    # Write data from server on .txt files
    # file_init_posNED = open("init_posNED.txt", "w")
    # file_waypointsNED = open("WaypointsNED.txt", "w")
    # for i in range(real_world_parameters.droneNo):
    #     file_init_posNED.write(''.join('{}, {}').format(init_posNED[i][0], init_posNED[i][1]))
    #     file_init_posNED.write('\n')
    #     for j in range(len(WaypointsNED[i])):
    #         file_waypointsNED.write(''.join('{}, {}, {}').format(i, WaypointsNED[i][j][0], WaypointsNED[i][j][1]))
    #         file_waypointsNED.write('\n')
    # file_waypointsNED.close()
    # file_init_posNED.close()

    # """Read data from server"""
    # init_posNED_server = []
    # WaypointsNED_server = [[] for _ in range(real_world_parameters.droneNo)]
    # WaypointsNED_helper = []
    # init_posNED_helper = []
    #
    # file_data = open('init_posNED_50000_large.txt', 'r')
    # lines = file_data.readlines()
    # for line in lines:
    #     init_posNED_helper.append(line.strip().split(", "))
    # for WP in init_posNED_helper:
    #     init_posNED_server.append([float(WP[0]), float(WP[1])])
    #
    # file_data_ = open('WaypointsNED_50000_large.txt', 'r')
    # lines = file_data_.readlines()
    # for line in lines:
    #     WaypointsNED_helper.append(line.strip().split(", "))
    # for WP in WaypointsNED_helper:
    #     WaypointsNED_server[int(WP[0])].append([float(WP[1]), float(WP[2])])

    # Choose your simulator ..
    if AirSim:
        run_airsim(real_world_parameters, init_posNED, WaypointsNED)

    elif AirLearning:
        # run_airlearning(real_world_parameters, init_posNED, WaypointsNED)
        run_airlearning(real_world_parameters, init_posNED_server, WaypointsNED_server)



def initializeDARPGrid(randomInitPos, l, m, initialPos, megaNodes, theta, shiftX, shiftY, droneNo):
    DARPgrid = megaNodes[:, :, 2].astype(int)

    if randomInitPos:
        # Add drones in random initial positions
        c = 0
        while c < droneNo:
            # Initial positions of drones
            ind1 = random.randrange(0, l, 1)
            ind2 = random.randrange(0, m, 1)
            if DARPgrid[ind1][ind2] == 0:
                DARPgrid[ind1][ind2] = 2
                c += 1
    else:
        # Add drones in the closest mega-node
        i1 = 0
        i2 = 0

        # Convert initial positions from WGS84 to NED
        initPosNED = ConvCoords(geoCoords, geoObstacles).convWGS84ToNED(initialPos)

        # Rotate and shift initial positions
        rotate = Rotate()
        rotate.setTheta(theta)
        initialPosNED = rotate.rotatePolygon(initPosNED)
        for i in range(len(initialPos)):
            initialPosNED[i][0] += shiftX
            initialPosNED[i][1] += shiftY

        for i in range(len(initialPosNED)):
            minDist = sys.float_info.max
            for j in range(l):
                for k in range(m):
                    distance = Dist.euclidean(initialPosNED[i], [megaNodes[j][k][0], megaNodes[j][k][1]])
                    if distance < minDist and megaNodes[j][k][2] == 0:
                        minDist = distance
                        i1 = j
                        i2 = k

            DARPgrid[i1][i2] = 2
            megaNodes[i1][i2][2] = -1

    return DARPgrid


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-optimization',
        action='store_true',
        help='Run Using Optuna')
    argparser.add_argument(
        '-number_of_trials',
        type=int,
        default=1000,
        help='Number of trials that the optimization will run for.')
    argparser.add_argument(
        '-AirSim',
        action='store_true',
        help='Run Simulation Using AirSim')
    argparser.add_argument(
        '-AirLearning',
        action='store_true',
        help='Run Simulation Using AirLearning-UE4')

    args = argparser.parse_args()
    run_experiment(args.optimization, args.number_of_trials, args.AirSim, args.AirLearning)
# cProfile.run('run_experiment(args.optimization, args.AirSim)', sort='tottime')
