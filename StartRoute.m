
clear  

%% Add Paths
addpath(genpath('Data/PowertrainFitData'));
addpath(genpath('Casadi'));
addpath(genpath('Data/SavedDrivingMissions'));


%% Load Driving Mission
load('Data/SavedDrivingMissions/C2C_2.mat')
v=[];
%load('Data/Vehicles/Paper_ID3.mat')

%% Set Optimizer Options
options.twoGears = 0;
options.twoMotors = 0;
options.solver.limitOverallRoute = 0;
options.solver.splitGbMap = 1;
options.solver.splitMotMap = 1;
options.solver.minTorqueRecu = 0;
options.plot.results = 1;

%% Set variable vehicle Parameters
 aux=205;
stSOC = 80;  % Combination of high SOC and Speed Limit is hard to calculate

%% Generate Driving Mission Object
OFR=ClassDefs.OptiFrameRoute('t',v,options);

%% Optimize Driving Mission
OFR.optiDrivingMission(dm,aux,stSOC);

%% Plot results
OFR.plotFun();

%% Show results of post processed quasi static simulation
OFR.resQs.Energy

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Optimize with customized values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

options.twoMotors = 1;
load('Data/Vehicles/Paper_ID3_2M.mat')

%% Generate Driving Mission Object
OFR2=ClassDefs.OptiFrameRoute('t',veh2M,options);

%% Edit weighting of objective function
OFR2.defineWeightParam(1, 2, 0.1);   %Weight on jerk, acceleration, energy (other weighting parameters stay default)

%% Edit constant driving parameters
OFR2.defineConstParam(5, -10, 5);   %  maximum acceleleration during traction, maximum acceleleration during braking, maximum jerk

%% Optimize Driving Mission
OFR2.optiDrivingMission(dm,aux,stSOC);

%% Plot results
OFR2.plotFun();

%% Show results of post processed quasi static simulation
OFR2.resQs.Energy





