# Optimization documentation

This README was created as a result of the **Cython optimization** performed to the Path Planner workload.

## Introduction

This README describes the required steps to run the workload with the added modifications to optimize its performance. The full optimization process is explained in the report available here (PENDENT).

## Cython Modifications

The performed modifications to the original code are the following:

- Transformed all the computation section of the *strictlyInPoly* method to cython
- Replace the *for* loop inside the *checkInPolyAndObstacle* method by a *while* loop
- Removed the subNodes matrix and reduced the megaNodes to only two axis 
- Modified the requirements.txt file
- Added a *.gitignore* file
- Removed part of the visualization process for an easier integration with the server where the experiments are performed

## How to set up

The required steps to set up the modified code are the following:

- Install python3.9
- Perform the command `sudo apt-get install python3.9-dev`
- Install the packages inside the *requirements.txt* file
- Compile the cython code with the command `cd RealWorld\handleGeo && python3 setup.py build_ext --inplace`

## How to run

The required steps to run the modified code are the following:

- Run the command `python main.py`
