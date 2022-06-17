import os

"""Class for simulation worlds parameter parser"""


class simulation_world:
    def __init__(self):
        self.simulation_world_parameters()

    def simulation_world_parameters(self):
        self.velocity = 10  # m/s
        self.distance_threshold = 1  # m
        self.corner_radius = self.velocity + 1  # m

        # Write execution time - flight time
        file_name_exec_time = "Flight_time.txt"
        completeName_exe_time = os.path.join('', file_name_exec_time)
        self.file_exec_time = open(completeName_exe_time, "w")

        """ Energy_Consumed.txt & Distance_Traveled.txt are only for AirLearning-UE4"""
        # Write energy consumption
        file_name_power = "Energy_Consumed.txt"
        completeName_power = os.path.join('', file_name_power)
        self.file_power_consumed = open(completeName_power, "w")

        # Write distance traveled
        file_name_distance = "Distance_Traveled.txt"
        completeName_distance = os.path.join('', file_name_distance)
        self.file_distance_traveled = open(completeName_distance, "w")

    def get_mission_parameters(self):
        return self.velocity, self.distance_threshold, self.corner_radius
