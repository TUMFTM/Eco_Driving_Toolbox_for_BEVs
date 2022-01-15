# Eco-Driving-Toolbox for Battery Electric Vehicles
 
This repository provides a toolbox for tank-to-distance-eco-driving for battery electric vehicles. Thereby, the losses of battery, power electronics, motor, and gearbox are 
incorporated and minimized. Based on tabulated data (loss maps) of the single components meta-models can be generated, which can be used within the optimization. 
The speed profile is optimized according to the users preferences (jerk, acceleration, energy, distance to the leading vehicle, ...). 
A route optimization (R) for optimizing die speed along a given route without a leading vehicle as well as an moving horizon 
speed planner (M) to simulate an adaptive cruise control is included. The post-processing allows the simulation of the optimization results based on the original tabulated loss-models.
 
 
## Requirements
- Matlab (tested on version R2021a)
- Casadi (tested on version v3.5.5; https://web.casadi.org/get/)
 
## Installation
Copy downloaded ``Casadi.zip`` into ``\Casadi``-Folder and extract it there
 
## Test Model
Two examples are provided:
- StartRoute optimizes the speed along a given route with given speed limits
- StartMovingHorizon starts a car-following scenario in which the leading vehicle drives the WLTC
 
## Folder Structure
The code is structured as follows
- '+ClassDefs' includes all class definitions
- '+HelpFun' includes functions that are needed at different locations of the code and the fitting tool
- 'Casadi' is a default empty folder. Please add Casadi in here
- 'Data' includes all required data as loss maps and their fits or saved driving missions
- 'PreProcessing' includes the skripts to generate the fitted loss models of the Volkswagen ID3
- 'DataExtern' includes data from extern sources
 
## General Program Structure
![Alt text](+ClassDefs/Structure.png?raw=true "Program Structure")
 
## Program Options
The program offers different options, depending on the optimization mode:
|  Name                |                     Possibilities                 |         Default Value |  Optimization Mode | Explanation |
| :-----------------|:-------------------------------------:|:----------------:|:--------------------------:|:-------------------|
| options.twoGears |  0/1      |        0   |    R+M|  0 if vehicle has single-speed transmission, 1 if vehicle has two-speed transmission  |
| options.twoMotors  |  0/1      |      0   |   R+M|    0 if vehicle has single driven axle, 1 if vehicle has two driven axles |
| options.solver.limitOverallRoute |   0/1    |   0   |  R|  1 if total summed jerk and acceleration should be limited by a value over whole optimization. Requires additional parameters 'param_variable.a_max_sq_sum' and 'param_variable.j_max_sq_sum' for optimization |
| options.maxDistance |   0/1    |   0   |   M| 1 if maximal distance to leading vehicle should be limited. 
options.solver.splitGbMap  |   0/1    |   1  |   R+M| 0 if continuous meta-models of gearbox should be used, 1 if split meta-models should be used. Split meta-model generate better results, continuous are slightly faster.
options.solver.splitMotMap  |   0/1    |   1  |   R+M| 0 if continuous meta-models of motor should be used, 1 if split meta-models should be used. Split meta-model generate better results, continuous are slighlty faster.
options.solver.minTorqueRecu  |   0/1    |   0  |   R+M| 1 if recuperation should be limited additionally to maximum motor toque over speed
options.carEstimater |   'V2V'/'a'   |   'V2V'  |   M| 'V2V': following vehicle knows the velocity of the leading vehicle for the prediction time. 'a': following vehicle estimates the velocity of the leading vehicle for the prediction time.
options.plot.results =  |  0/1    |   0  |   R+M| 1 if some plots at the end of simulation should appear
options.plot.dynamic =  |  0/1/2    |   0  |   M|  0 no dynamic plot, 1 dynamic plot of speed, 2 dynamic plot of speed and torque. Dynamic plot makes simulation slower.
 
 
## Required Structure of a 'Driving Mission'
The driving mission (dm) sets the boundary conditions of the driving task. The data is given within a struct. Its size depend on the optimization mode. For the route optimization, the stuct must include:
| Variable name  |  Dimension     |          Explanation |
|:----------------|:---------------:|:------------------|
| t                       |  m x 1               |Time vector  
| ubv                       |  m x 1         |Upper bound velocity (time-dependent)  
| lbv                       |  m x 1          |Lower bound velocity (time-dependent)  
| ubs                       |  m x 1         |Upper bound distance (time-dependent)  
| lbs                       |  m x 1          |Lower bound distance (time-dependent)  
| a_init_lb                |  1 x 1          |Maximum acceleration at start point
| a_init_ub                |  1 x 1          |Minimum acceleration at start point
| a_next_param       |  1 x 1          |Acceleration that should be continued with in the first step beyond the optimization
| a_last_param       |  1 x 1          |Acceleration that was driven before the first step of optimization
| v_max       |  p x 1          | Maximum speed starting at the corresponding distance of v_max_s (spatial-dependent)
| v_max_s      |  p x 1          | Distance of the speed limits, defined in v_max
 
For the moving horizon optimization, the stuct must include:
| Variable name  |  Dimension     |          Explanation |
|:----------------|:---------------:|:------------------|
| t                       |  m x 1               |Time vector  
| lv_v                       |  m x 1         |Velocity of leading vehicle at every time point of t
| lv_s                       |  m x 1         |Location of leading vehicle at every time point of t
| ev_v_init                |  1 x 1         |Initial velocity of ego-vehicle
| v_max       |  p x 1          | Maximum speed starting at the corresponding distance of v_max_s (spatial-dependent)
| v_max_s      |  p x 1          | Distance of the speed limits, defined in v_max
 


## Publication
LINK
DOI: 10.1

Contact person: [Alexander Koch](mailto:alexander.koch@tum.de)
 
 
 
 


