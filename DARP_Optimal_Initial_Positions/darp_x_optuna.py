import argparse
import numpy as np
import os
import matplotlib as plt
import optuna
from optuna.samplers import TPESampler
import sys
sys.path.append('DARP')
from multiRobotPathPlanner import MultiRobotPathPlanner
from optuna.structs import TrialPruned

class optimize():
    def __init__(self, rows, cols, number_of_trials, nep, portions, obstacles_positions, vis, num_drones):
        # DARP Parameters
        self.number_of_drones = num_drones
        self.cols = cols
        self.rows = rows
        self.nep = nep
        self.portions = portions
        self.obstacles_positions = obstacles_positions
        self.obstacles_coords = obstacles_positions
        self.vis = vis
        
        # Optimization Parameters
        self.samplers = [optuna.samplers.TPESampler()]
        self.number_of_trials = number_of_trials
        self.results = []
        self.best_avg = sys.maxsize
        self.all_instances = []

    def optimize(self):        
        for sampler in self.samplers:
            study = optuna.create_study(
                study_name="study",
                directions=["minimize"],
                sampler=sampler,
                pruner=optuna.pruners.MedianPruner()
            )
            study.optimize(self.objective, n_trials=self.number_of_trials)
            self.results.append(study)
        
        for result in self.results:
                best_avg = sys.maxsize
                for t in result.best_trials:
                    if self.all_instances[t.number].best_case.avg < best_avg:
                        best_avg = self.all_instances[t.number].best_case.avg
                        self.best_trial = self.all_instances[t.number]
    
    def objective(self, trial):
        positions = []

        for i in range(self.number_of_drones):
            positions.append(trial.suggest_int(f"p{i}", 0, self.rows*self.cols-1))

        if len(positions) != len(set(positions)):
            self.all_instances.append("Pruned")
            raise TrialPruned()
        
        for obstacle in self.obstacles_positions:
            for p in positions:
                if p == obstacle:
                    self.all_instances.append("Pruned")
                    raise TrialPruned()

        multiRobotPathPlanner_instance = MultiRobotPathPlanner(self.rows, self.cols, self.nep, positions, self.portions, self.obstacles_coords, self.vis)
        self.all_instances.append(multiRobotPathPlanner_instance)

        if not multiRobotPathPlanner_instance.DARP_success:
            raise TrialPruned()
        else:
            return max(multiRobotPathPlanner_instance.best_case.turns)


    def export_results(self):
        counter = 1
        for result in self.results:
            print(f"Study{counter}")
            print("Number of finished trials: ", len(result.trials))
            
            if self.obstacles_positions == []:
                obs_flag = "No_obstacles"
            else:
                obs_flag = "With_obstacles"

            if not os.path.exists(f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}'):
                os.mkdir(f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}')

            best_avg = sys.maxsize
            for t in result.best_trials:
                print(f"Best trial: {t.number}")
                print(f'Best value: {t.values[0]}')
                print(f'Best param: {t.params}')
                
                with open(f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}/{result.sampler}.txt', "a") as f1:
                    f1.write(f"Best trial: {t.number}\n")
                    f1.write(f"Best value: {t.values[0]}\n")
                    f1.write(f"Best parameters:")
                    for param, val in t.params.items():
                        f1.write(f"{val}")
                        f1.write(" ")
                    f1.close()

            params = []
            for param in result.trials[0].params.items():
                params.append(param[0])

            optuna.visualization.plot_contour(result, params=params)

            all_results = dict()
            for i in range(len(result.trials)):
                temp_all_results = list()
                for index, value in result.trials[i].params.items():
                    temp_all_results.append(value)
                
                temp_all_results = tuple(temp_all_results)
                
                if result.trials[i].values != None:
                    all_results[temp_all_results] = result.trials[i].values[0]
                else:
                    all_results[temp_all_results] = None

            with open(f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}/all_results.txt', "a") as f1:
                f1.write(f'Sampler:{result.sampler}\n')
                
                for pos, res in all_results.items():
                    f1.write(f'{pos}, {res}\n')
                
                f1.close()

            # fig = optuna.visualization.plot_contour(result, params=params)
            # fig.write_image(f'x = {self.rows}, y = {self.cols}, num_drones = {self.number_of_drones}, obstacles = {obs_flag}/{result.sampler}_contour.png')

            counter += 1


if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-grid',
        default=(10, 10),
        type=int,
        nargs=2,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-obs_pos',
        default=[],
        nargs='*',
        type=int,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-nep',
        action='store_true',
        help='Not Equal Portions shared between the Robots in the Grid (default: False)')
    argparser.add_argument(
        '-portions',
        default=[0.2, 0.3, 0.5],
        nargs='*',
        type=float,
        help='Portion for each Robot in the Grid (default: (0.2, 0.7, 0.1))')
    argparser.add_argument(
        '-vis',
        action='store_true',
        help='Visualize results (default: False)')
    argparser.add_argument(
        '-num_drones',
        default=[2],
        type=int,
        nargs=1,
        help='Insert desired number of drones')
    args = argparser.parse_args()

    rows, cols = args.grid
    num_drones = args.num_drones[0]

    for obstacle in args.obs_pos:
        if obstacle < 0 or obstacle >= rows*cols:
            print("Obstacles should be inside the Grid.")
            sys.exit(3)

    portions = []

    if args.nep:
        for portion in args.portions:
            portions.append(portion)
    else:
        for drone in range(num_drones):
            portions.append(1/num_drones)

    if num_drones != len(portions):
        print("Portions should be defined for each drone")
        sys.exit(4)

    s = sum(portions)
    if abs(s-1) >= 0.0001:
        print("Sum of portions should be equal to 1.")
        sys.exit(1)

    importance = False

    print("\nInitial Conditions Defined:")
    print("Grid Dimensions:", rows, cols)
    print("Robot Number:", num_drones)
    print("Portions for each Robot:", portions, "\n")

    if rows == 10 and cols == 10:
        number_of_trials = 2000
    elif rows == 15 and cols == 20:
        number_of_trials = 5000
    else:
        number_of_trials = 8000

    optimization = optimize(rows, cols, number_of_trials, args.nep, portions, args.obs_pos, args.vis, num_drones)
    optimization.optimize()
    optimization.export_results()
