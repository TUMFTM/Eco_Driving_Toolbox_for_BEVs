clear  

%% Add Paths
addpath(genpath('Data/PowertrainFitData'));
addpath(genpath('Casadi'));
addpath(genpath('Data/SavedDrivingMissions'));

%% Load Driving Mission
 load('MPC_long_PAPER.mat')


%% Set Optimizer Options
options.twoGears = 0;
options.twoMotors = 0;
options.maxDistance = 0;
options.solver.splitGbMap = 1;
options.solver.splitMotMap = 1;
options.solver.minTorqueRecu = 0;
options.carEstimater = 'V2V';  %%'V2V' od 'a'
options.plot.results = 1; % Plot results of quasi static simulation
options.plot.dynamic = 2;  %0,1,2 slows down optimization


%% Set variable vehicle Parameters
 aux=205;
stSOC = 80;


%% Generate Driving Mission Object
OFM = ClassDefs.OptiFrameMovingHorizon('t',[],options);

%% Set Driving Behavior
OFM.defineWeightParam(20, 0, 4, 0.0001, 0.000001, 0, 0, 4,  0.01);  %(w_j, w_a, w_E, w_r, w_r_br, w_v, w_v_lim, w_vEnd,  w_s)

%% Optimize Driving Mission
OFM.optiDrivingMission(dm, aux, stSOC);   
%% Plot results
OFM.plotFun();

%% Show results of post processed quasi static simulation
OFM.resQs.Energy


