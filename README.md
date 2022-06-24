# RealWorld2AirSim-DARP

## Description 

This project is a multi-robot coverage path planning module(mCPP) that utlizes [DARP](https://github.com/alice-st/DARP/tree/main) algorithm to cover an area of interest with prior-defined NoFly zones/obstacles in WGS84 system. The implemented algorithm is also capable of providing optimal initial positions for a team of mobile robots in order to efficiently cope with real-life multi-UAV coverage missions, utilizing [DARP_Optimal_Initial_Positions](https://github.com/alice-st/DARP_Optimal_Initial_Positions/tree/main). RealWorld2AirSim-DARP can provide paths through turn-waypoints which can be used either in a real world or in a semi-realistic scenarios by utilizing both [AirSim](https://microsoft.github.io/AirSim/) and [AirLearning](https://github.com/harvard-edge/AirLearning) simulators with an autonomous navigation scheme.

An on-line instance of an end-to-end mission planner utilizing this mCPP module can be found here:
http://choosepath.ddns.net/

## Input/Output:

The algorithm receives as input the following:

- The number of robots/vehicles
- The desired sidelap (sidelap corresponds to the desired ovelap between two sequential images in percentage)
- A polygon Region of Interest (ROI), formatted in WGS84 coordinate system
- A set of obstacles (polygons formatted in WGS84 coordinate system) inside the ROI (optional)
- A boolean variable named pathsStrictlyInPoly, to select mode between (paths strictly in poly/better coverage)
- A bolean variable named OptimalInitPos, to select mode between (optimal initial positions True/False in coverage paths)
- The initial positions of the vehicles (optional - if OptimalInitPos is False, random will be used instead | Note that the number of the initial positions should always be the same as the number of robots/vehicles)
- The desired percentages for proportional area allocation (optional - if not provided, equal will be used instead | Note that the number of the percentages should always be the same as the number of robots/vehicles and their sum should be 1)

As an output, the algorithm provides set of waypoints (path), for each vehicle involved in the mission, in order to cooperatively completely cover the ROI.

## How to run:
Inputvariables_choosepath.json countains input paramaters with the same order described above.

In a terminal navigate to \RealWorld2AirSim-DARP and run:

### 1st 

```
python main.py
```

In this way, the module will provide and visualize coverage paths according to the input variables included in the JSON file.

![](https://github.com/emmarapt/RealWorld2AirSim-DARP/blob/122aa9e8e71ea13ba61543f5fa953488363abdb7/images/Random_Initial_Positions.png)

### 2nd

```
python main.py -optimization
```

In this way, the module will provide and visualize coverage paths according to the input variables included in the JSON file by utilizing the [DARP_Optimal_Initial_Positions](https://github.com/alice-st/DARP_Optimal_Initial_Positions/tree/main) submodule to provide optimal initial positions for the available robots/vehicles.


![](https://github.com/emmarapt/RealWorld2AirSim-DARP/blob/main/images/Optimal_Initial_Positions.png)

In case you change the number of trials that the optimization will run for, please run:

```
python main.py -optimization -number_of_trials
```

### 3rd

```
python main.py -AirSim
```

In this way, the module will provide, visualize and use AirSim simulator for autonomous UAV-based coverage path planning operations. The number of UAVs will be based on the JSON file and automatically spawn and follow the extracted coverage paths inside AirSim. Please note, that any AirSim environment must be launched before in order to have a successfull communication with the UAV client.

> -optimization -number_of_trials arguments can be used in combination with -AirSim

### 4th

```
python main.py -AirLearning
```

In this way, the module will provide, visualize and use Air Learing environment generator for autonomous UAV-based coverage path planning operations utilizing flight time, distance and energy consumption models. The number of UAVs must be added manually as AirLearning-UE4 does not provide a simAddVehicle API. Please also note, that Air Learing environment generator must be launched before in order to have a successfull communication with the UAV client.

> -optimization -number_of_trials arguments can be used in combination with -AirLearning


## AirSim Demo:

By choosing to run AirSim, each UAV will automatically follow the extracted waypoints to cooperatively completely cover a ROI. Here is a video demonstration of using AirSim along with UAVs for coverage path planning operations.

![](https://github.com/emmarapt/RealWorld2AirSim-DARP/blob/main/gifs/AirSim.gif)


## Air Learning Environment Generator

Air Learning environment generator is built on top of UE-4 game engine by using Microsoft's [AirSim](https://microsoft.github.io/AirSim/) plugin for aerial robot models and physics. The instructions of installing Air Learning can be found [here](https://github.com/harvard-edge/airlearning-ue4/tree/master).

### Quality metrics of multi-UAV missions
- Energy consumption per UAV
- Distance travelled per UAV
- Flight time per UAV

### Energy Consumption model for UAVs

> To use the energy consumption model, navigate to ..\Epic Games\UE_4.18\Engine\Binaries\Win64 and in your AirSim settings.json file add the following:

```
"EnergyModelSettings": {
    "mass": 2,
    "mass_coeff": 0.22,
    "vxy_coeff": -1.526,
    "axy_coeff": 3.934,
    "vxy_axy_coeff": 0.968,
    "vz_coeff": 18.125,
    "az_coeff": 96.613,
    "vz_az_coeff": -1.085,
    "one_coeff": 443.9,
    "vxy_wxy_coeff": 0
  }
```

### Air Learning Demo:

By choosing to run AirLearning, each UAV will automatically follow the extracted waypoints to cooperatively completely cover a ROI. Quality metrics described above could be also used to evaluate each UAV's mission. Here is a video demonstration of using Air Learning along with UAVs for coverage path planning operations.

![](https://github.com/emmarapt/RealWorld2AirSim-DARP/blob/main/gifs/AirLearning.gif)


