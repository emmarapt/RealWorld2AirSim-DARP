"""Class for utilizing AirLearning-UE4 with AirSim plugin for Autonomous UAV-based Coverage Path Planning operations"""

from SimWorld.simulation_world_parameter_parser import simulation_world
import sys

sys.path.append("D:\\airlearning-ue4\AirSim\PythonClient")
import airsim
import threading
import numpy as np
import time


def run_airlearning(real_world_parameters, init_posNED, WaypointsNED):
    """AirLearning-UE4 does not provide simAddVehicle API - Add manually the number of drones """
    # connect to Drone client
    # client = airsim.MultirotorClient()
    client = airsim.MultirotorClient(ip="127.0.0.1")
    client.confirmConnection()

    # enable Drone1 - default drone when opening AirSim
    client.enableApiControl(True, "Drone1")
    client.armDisarm(True, "Drone1")

    # enable Drone1 - default drone when opening AirSim
    client.enableApiControl(True, "Drone2")
    client.armDisarm(True, "Drone2")

    # enable Drone1 - default drone when opening AirSim
    client.enableApiControl(True, "Drone3")
    client.armDisarm(True, "Drone3")

    simulation_world_parameters = simulation_world()
    velocity, distance_threshold, corner_radius = simulation_world_parameters.get_mission_parameters()

    def GetMultirotorState(DroneName):
        getdata = client.getMultirotorState(DroneName)
        # time.sleep(0.1)  # sleep to avoid BufferError
        return getdata

    class Thread_class(threading.Thread):
        def __init__(self, running):
            threading.Thread.__init__(self)
            self.running = running
            self.daemon = True
            self.start()

        def run(self):
            while self.running:
                self.update_path()

        def update_path(self):
            global getpose
            global resulting_path
            # global ned_dist

            global turn

            for i in range(real_world_parameters.droneNo):
                try:

                    # get drone states
                    getpose = GetMultirotorState('Drone{}'.format(i + 1))

                    current_ned = np.array(
                        (getpose.kinematics_estimated.position.x_val, getpose.kinematics_estimated.position.y_val))
                    nextWP_ned = np.array((path_drone[i][0].x_val, path_drone[i][0].y_val))
                    ned_dist = np.linalg.norm(current_ned - nextWP_ned)

                    # print(
                    # 	f'Curent position: [{getpose.kinematics_estimated.position.x_val}, {getpose.kinematics_estimated.position.y_val}], '
                    # 	f'Next waypoint: [{path_No1[0].x_val}.{path_No1[0].y_val}], Distance: {ned_dist}')

                    if ned_dist < distance_threshold:
                        del path_drone[i][0]
                        print("WPs to go for Drone{}:".format(i + 1), len(path_drone[i]))

                    if len(path_drone[i]) == 0:
                        """ -------------------------- Get Trip status -------------------------- """
                        get_Trip_Stats_end_path = client.getTripStats('Drone{}'.format(i + 1))


                        # get_time = client.getMultirotorState('Drone{}'.format(i + 1))
                        end_date_time = datetime.fromtimestamp(getpose.timestamp // 1000000000)
                        print('Last WP !! --- Drone{} ends its path at {} ---'.format(i + 1,
                                                                                      end_date_time - start_date_time))

                        # Execution time
                        Execution_time = end_date_time - start_date_time

                        # write execution time in .txt
                        simulation_world_parameters.file_exec_time.write(
                            ''.join('Overall execution time for Drone{} is {} sec'.format(i + 1,
                                                                                          get_Trip_Stats_end_path.flight_time - get_Trip_Stats_start_path.flight_time)))
                        simulation_world_parameters.file_exec_time.write(''.join('\n'))

                        simulation_world_parameters.file_power_consumed.write(
                            ''.join('Overall energy consumption for Drone{} is {} watt'.format(i + 1,
                                                                                               get_Trip_Stats_end_path.energy_consumed - get_Trip_Stats_start_path.energy_consumed)))
                        simulation_world_parameters.file_power_consumed.write(''.join('\n'))

                        simulation_world_parameters.file_distance_traveled.write(''.join(
                            'Overall distance traveled for Drone{} is {} meters'.format(i + 1,
                                                                                        get_Trip_Stats_end_path.distance_traveled - get_Trip_Stats_start_path.distance_traveled)))
                        simulation_world_parameters.file_distance_traveled.write(''.join('\n'))

                    if ned_dist < corner_radius:  # Corner Radius MODE
                        resulting_path = client.moveOnPathAsync(path_drone[i], 1, 500,
                                                                airsim.DrivetrainType.ForwardOnly,
                                                                airsim.YawMode(False, 0),
                                                                vehicle_name='Drone{}'.format(i + 1))
                        # print("Corner Radius mode for Drone {}".format(i + 1))
                        turn = True  # Drone is in Turn
                    else:

                        resulting_path = client.moveOnPathAsync(path_drone[i], velocity, 500,
                                                                airsim.DrivetrainType.ForwardOnly,
                                                                airsim.YawMode(False, 0),
                                                                vehicle_name='Drone{}'.format(i + 1))
                except:
                    pass

    z = - real_world_parameters.altitude
    # store waypoints to list
    global path_drone
    path_drone = [[] for _ in range(real_world_parameters.droneNo)]
    init_pos_drone = []

    # ------------------ Read path from mCPP -----------------------------------------------------------------------

    for i in range(real_world_parameters.droneNo):
        init_pos_drone.append(airsim.Vector3r(init_posNED[i][0], init_posNED[i][1], -real_world_parameters.altitude))
        for j in range(len(WaypointsNED[i])):
            # path_No1.append(airsim.Vector3r(WaypointsNED[0][0][i][0] - 120, WaypointsNED[0][0][i][1] - 70, z))
            #path_drone[i].append(airsim.Vector3r(WaypointsNED[i][0][j][0], WaypointsNED[i][0][j][1], z))
            path_drone[i].append(airsim.Vector3r(WaypointsNED[i][j][0], WaypointsNED[i][j][1], z))


    airsim.wait_key('Press any key to takeoff')
    for i in range(real_world_parameters.droneNo):
        f1 = client.takeoffAsync(vehicle_name="Drone{}".format(i + 1)).join()

    print("make sure we are hovering at {} meters...".format(-z))
    for i in range(real_world_parameters.droneNo):
        hovering = client.moveToZAsync(z, 5, vehicle_name='Drone{}'.format(i + 1)).join()

    # send missions to drones / send initial positions to the drones
    for i in range(real_world_parameters.droneNo):
        # mission = client.moveOnPathAsync(path_drone[i], velocity, 500, airsim.DrivetrainType.ForwardOnly,
        #                                  airsim.YawMode(False, 0), 3 + 3 / 2, vehicle_name='Drone{}'.format(i + 1))

        initial_pos_on_AirSim = client.moveToPositionAsync(path_drone[i][0].x_val, path_drone[i][0].y_val,
                                                           path_drone[i][0].z_val, velocity, 500,
                                                           airsim.DrivetrainType.ForwardOnly,
                                                           airsim.YawMode(False, 0), 3 + 3 / 2,
                                                           vehicle_name='Drone{}'.format(i + 1))
    # Wait until first WP is reached
    movetopath = True
    # movetopath = [True for _ in range(droneNo)]
    movetopath_status = [False for _ in range(real_world_parameters.droneNo)]
    while movetopath:
        for i in range(real_world_parameters.droneNo):

            if movetopath_status[i] is False:

                get_initial_pose = client.getMultirotorState('Drone{}'.format(i + 1))

                current_ned = np.array(
                    (get_initial_pose.kinematics_estimated.position.x_val,
                     get_initial_pose.kinematics_estimated.position.y_val))
                nextWP_ned = np.array((path_drone[i][0].x_val, path_drone[i][0].y_val))
                ned_dist = np.linalg.norm(current_ned - nextWP_ned)

                if ned_dist < distance_threshold:
                    del path_drone[i][0]
                    movetopath_status[i] = True

                    # movetopath = [False if x == i else x for x in movetopath]
                    print("Drone{}: WPs to go:".format(i + 1), len(path_drone[i]))

                    movetopath = False in (elem is True for elem in movetopath_status)
                    onpath = all(movetopath_status)

                    # print("status", movetopath_status)
                    # print("moveonpath", movetopath)
                    # print("onpath", onpath)
                    # onpath = [True in (elem is False for elem in movetopath) for _ in range(droneNo)]

    for i in range(real_world_parameters.droneNo):
        get_time = client.getMultirotorState('Drone{}'.format(i + 1))
        from datetime import datetime

        start_date_time = datetime.fromtimestamp(get_time.timestamp // 1000000000)
        print(" --- First WP !! --- Drone{} starts to follow the path at {} ---".format(i + 1,
                                                                                        start_date_time))

        get_Trip_Stats_start_path = client.getTripStats('Drone{}'.format(i + 1))

    # Start Thread for updating the path -- Background process
    Thread_class(True)

    moveonpath_status = [False for _ in range(real_world_parameters.droneNo)]
    while onpath:
        for i in range(real_world_parameters.droneNo):

            if len(path_drone[i]) == 0:
                moveonpath_status[i] = True
                onpath = not all(moveonpath_status)

    time.sleep(0.5)
    # Thread_class(False)
    print('\n')
    # END
    # airsim.wait_key('Press any key to reset to original state')
    # for i in range(real_world_parameters.droneNo):
    #     client.armDisarm(False, "Drone{}".format(i + 1))
    #     client.reset()
    #
    #     # that's enough fun for now. let's quit cleanly
    #     client.enableApiControl(False, "Drone{}".format(i + 1))
    print(
        " -------------- Overall execution for every drone is written in the Flight_time.txt ------------------- ")
    print(
        " -------------- Overall energy consumption for every drone is written in the Energy_Consumed.txt ------------------- ")
    print(
        " -------------- Overall distance traveled for every drone is written in the Distance_Traveled.txt ------------------- ")

    simulation_world_parameters.file_exec_time.close()
    simulation_world_parameters.file_power_consumed.close()
    simulation_world_parameters.file_distance_traveled.close()
