"""Class for utilizing AirSim for Autonomous UAV-based Coverage Path Planning operations"""

from simulation_world_parameter_parser import simulation_world
from datetime import datetime
import time
import threading
import airsim
import numpy as np


def run_airsim(real_world_parameters, init_posNED, WaypointsNED):

    # connect to the AirSim simulator
    # connect to Drone client

    client = airsim.MultirotorClient()
    client.confirmConnection()

    # enable Drone1 - default drone when opening AirSim
    client.enableApiControl(True, "Drone1")
    client.armDisarm(True, "Drone1")


    simulation_world_parameters = simulation_world()
    velocity, distance_threshold, corner_radius = simulation_world_parameters.get_mission_parameters()

    def GetMultirotorState(DroneName):
        getdata = client.getMultirotorState(DroneName)
        time.sleep(0.1)  # sleep to avoid BufferError
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
                    # calculate power consumption for each drone
                    # rotors = GetRotorState('Drone{}'.format(i + 1))
                    # print(rotors)
                    # power_ = power_consumed.estimated_power(i, rotors)
                    # print("Current power consumption for drone{} is {}".format(i + 1, power_))

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
                        # get_time = client.getMultirotorState('Drone{}'.format(i + 1))
                        end_date_time = datetime.fromtimestamp(getpose.timestamp // 1000000000)
                        print('Last WP !! --- Drone{} ends its path at {} ---'.format(i + 1,
                                                                                      end_date_time - start_date_time))

                        # Execution time
                        Execution_time = end_date_time - start_date_time

                        # write execution time in .txt
                        simulation_world_parameters.file_exec_time.write(
                            ''.join('Overall execution time for Drone{} is {} sec'.format(i + 1, Execution_time)))
                        simulation_world_parameters.file_exec_time.write(''.join('\n'))

                        # # write power consumption in .txt
                        # final_estimated_power_ = power_consumed.final_estimated_power()
                        # file_power_consumed.write(''.join('Power Consumed for Drone{} is {} watt'.format(i + 1, final_estimated_power_[i])))
                        # file_power_consumed.write(''.join('\n'))

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





    # add the corresponding drones
    for i in range(real_world_parameters.droneNo - 1):
        done = client.simAddVehicle(vehicle_name="Drone{}".format(i + 2), vehicle_type="simpleflight",
                                    pose=airsim.Pose(airsim.Vector3r(2 + i, 0, 0), airsim.Quaternionr(0, 0, 0, 0)))

        client.enableApiControl(True, "Drone{}".format(i + 2))
        client.armDisarm(True, "Drone{}".format(i + 2))

    z = - real_world_parameters.altitude
    # # store waypoints to list
    global path_drone
    path_drone = [[] for _ in range(real_world_parameters.droneNo)]
    init_pos_drone = []

    # ------------------ Read path from mCPP ---------------------------------------------------------------------------

    for i in range(real_world_parameters.droneNo):
        init_pos_drone.append(airsim.Vector3r(init_posNED[i][0], init_posNED[i][1], -real_world_parameters.altitude))
        #for j in range(len(WaypointsNED[i][0])):
        for j in range(len(WaypointsNED[i])):
            #path_drone[i].append(airsim.Vector3r(WaypointsNED[i][0][j][0], WaypointsNED[i][0][j][1], z))
            path_drone[i].append(airsim.Vector3r(WaypointsNED[i][j][0], WaypointsNED[i][j][1], z))

    # # Visualize waypoints
    """ Press T for path visualization"""
    client.simPlotPoints(points=init_pos_drone, size=50, is_persistent=True)
    client.simPlotLineStrip(points=path_drone[0], color_rgba=[1.0, 2.0, 0, 0], thickness=30, is_persistent=True)
    client.simPlotLineStrip(points=path_drone[1], color_rgba=[1.0, -1.0, 1, 1], thickness=30, is_persistent=True)
    client.simPlotLineStrip(points=path_drone[2], color_rgba=[1, 0, 0, 1], thickness=30, is_persistent=True)

    # for i in range(real_world_parameters.droneNo):
    #     client.simPlotLineStrip(points=path_drone[i], color_rgba=[1.0, i - 2.0, i - 0, i], thickness=30, is_persistent=True)

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
    # movetopath = [True for _ in range(real_world_parameters.droneNo)]
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
                    # # start measuring execution time
                    #
                    # get_time = client.getMultirotorState('Drone{}'.format(i + 1))
                    # from datetime import datetime

                    del path_drone[i][0]
                    movetopath_status[i] = True

                    # movetopath = [False if x == i else x for x in movetopath]
                    print("Drone{}: WPs to go:".format(i + 1), len(path_drone[i]))

                    movetopath = False in (elem is True for elem in movetopath_status)
                    onpath = all(movetopath_status)

                    # print("status", movetopath_status)
                    # print("moveonpath", movetopath)
                    # print("onpath", onpath)
                # onpath = [True in (elem is False for elem in movetopath) for _ in range(real_world_parameters.droneNo)]

    for i in range(real_world_parameters.droneNo):
        get_time = client.getMultirotorState('Drone{}'.format(i + 1))

        start_date_time = datetime.fromtimestamp(get_time.timestamp // 1000000000)
        print(" --- First WP !! --- Drone{} starts to follow the path at {} ---".format(i + 1,
                                                                                        start_date_time))

    # Start Thread for updating the path -- Background process
    Thread_class(True)

    moveonpath_status = [False for _ in range(real_world_parameters.droneNo)]
    while onpath:
        for i in range(real_world_parameters.droneNo):
            if len(path_drone[i]) == 0:
                moveonpath_status[i] = True
                # End of execution time
                # end = time.time()

                # get_time = client.getMultirotorState('Drone{}'.format(i + 1))
                # end_date_time = datetime.fromtimestamp(get_time.timestamp // 1000000000)
                # print('Last WP !! --- Drone{} ends to follow the path at {} ---'.format(i + 1,
                #                                                                         end_date_time - start_date_time))
                #
                # # Execution time
                # Execution_time = end_date_time - start_date_time
                #
                # simulation_world_parameters.file_exec_time.write(
                #     ''.join('Overall execution time for Drone{} is {} sec'.format(i + 1, Execution_time)))
                # simulation_world_parameters.file_exec_time.write(''.join('\n'))
                #

                # onpath will be False if any Drone finishes its path
                onpath = not all(moveonpath_status)
            # onpath = [False if x == i else x for x in onpath]

    # END
    airsim.wait_key('Press any key to reset to original state')
    for i in range(real_world_parameters.droneNo):
        client.armDisarm(False, "Drone{}".format(i + 1))
        client.reset()
        # that's enough fun for now. let's quit cleanly
        client.enableApiControl(False, "Drone{}".format(i + 1))
    print(
        " -------------- Overall execution time for every drone is written in the Flight_time.txt ------------------- ")
    simulation_world_parameters.file_exec_time.close()
