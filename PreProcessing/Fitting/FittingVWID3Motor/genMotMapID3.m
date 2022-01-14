% Skript to generate Motor Map of ID3
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2021
% ------------
% Version: Matlab2021a
%-------------

clear

%Path
addpath(genpath('PreProcessing/Fitting/FittingTool'));

%% To get the inverter input power at coasting, we conducted experiments. However, data is scatterd, thus we fit data.

%Load Data
load('DataExtern\PowertrainComponents\Motor\ID3\coasting.mat')

% Fittingparameters Coasting
PARAcoast.polyX = 3;
PARAcoast.polyY = 0;
PARAcoast.Method = 'min SE';



% Do fitting
[fitFunCoast,~] = ...
    HelpFun.fittingEDT(coasting.n,zeros(size(coasting.n)),coasting.P,PARAcoast);

% Since 2268 rpm is smallest measurement point and fit is not valid below
% this rpm we replace the values by min of P_loss (only 27,5 Watt)
x_rpm=(1:1:14000);
P_loss = fitFunCoast(x_rpm,zeros(size(x_rpm)));
P_loss(x_rpm<min(coasting.n)) = min(P_loss);

% Evaluate
figure
title('Losses during coasting')
hold on
scatter(coasting.n,coasting.P)
plot(x_rpm,P_loss);
xlabel('Motor speed in rpm')
ylabel('Inverter Input Power')
legend('Measurements','Fit')


%% Generate Motor Map
% Motor Map based on Wassiliadis et. al - Quantifying the state of the art of electric powertrains in battery electric vehicles:
%Range, power, and lifetime from component to system level of the VW ID.3
load('DataExtern/PowertrainComponents/Motor/ID3/motorMap.mat')

% Calulate Losses based on efficiency
LOSS = (1-Eff_I2M).*RPM/60*2*pi.*Torque;

% Expand motor map by zero speed line
gradient =  (LOSS(2,:)  - LOSS(1,:)  )  ./   (RPM(2,:)  - RPM(1,:));
Loss_zero =  LOSS(1,:)  -      gradient .* RPM(1,:);
LOSS = [Loss_zero; LOSS];
RPM=[zeros(size(Loss_zero)); RPM];
Torque = [Torque(1,:); Torque];


% Expand motor map by zero torque line
loss_zero_T = interp1(x_rpm,P_loss,RPM(:,1),'linear','extrap');
LOSS = [loss_zero_T LOSS];
RPM=[RPM(:,1) RPM];
Torque = [zeros(size(loss_zero_T)) Torque];

% Evaluate
figure
hold on
mesh(RPM,Torque,LOSS)
title('Motor loss map')
xlabel('Motor speed in rpm')
ylabel('Motor torque in Nm')
zlabel('Motor losses in W')




%% Generate struct for Eco-Driving-Algorithm
% Nan handling
A_NaN = RPM;
A_NaN(~isnan(RPM)) = 1;

% create motor struct
motor.data.type = 'PMSM';
motor.data.n_rpm = RPM.*A_NaN;
motor.data.n_radps = RPM.*A_NaN/60*2*pi;
motor.data.n_rps = RPM.*A_NaN/60;
motor.data.T_Nm = Torque.*A_NaN;
motor.data.T_max_Nm = 309;
motor.data.n_max_rpm = 14000;
motor.data.p_max_vec_W = (max(motor.data.n_rpm.*motor.data.T_Nm*2*pi/60,[],2));
motor.data.p_max_kW = max(motor.data.p_max_vec_W)/ 1000;

% Since we can not seperate the electrical losses in iron losses, etc we
% assign them to the stator losses. However, calulation tools can differ
% the losses
motor.losses.iron_W = 0;
motor.losses.stator_W = LOSS;
motor.losses.inverter_W =0;
motor.losses.all_el_W = motor.losses.iron_W + motor.losses.stator_W + motor.losses.inverter_W;
% Mechanical motor losses
c_m = 8;
r_r = 0.0805;
l_m = 0.210;
T_loss =  c_m * 2 * r_r^3 *motor.data.n_radps*l_m;

% Add to motor struct
motor.losses.mech_W =  T_loss.*motor.data.n_radps;

% Cut maximal recuperation
motor.data.n_min_recu_radps = 320/60*2*pi;  % Aproximated from data




%% Fitting Motor Map for optimization


%% Fittingparameters

% Parameters for split fit
PARA.polyX = 5;
PARA.polyY = 3;
PARA.Method = 'min SRE';

% Parameters for continuous fit
PARA_co.polyX = 5;
PARA_co.polyY = 6;
PARA_co.Method = 'min SRE';


% reshape losses and operation points to vectors
x = reshape(motor.data.n_radps,numel(motor.data.n_rpm),1);
y =  reshape(motor.data.T_Nm,numel(motor.data.T_Nm),1);
z =  reshape(motor.losses.all_el_W,numel(motor.losses.all_el_W),1);

% Remove nans
x(isnan(z))=[];
y(isnan(z))=[];
z(isnan(z))=[];

x_co = [x; x];
y_co = [y; -y];
z_co = [z; z];

% Ineq
x_eq = [];
y_eq = [];
z_eq = [];

x_ineq = x;
y_ineq = -y;
z_ineq = z;


% Do fitting motor losses
% Split
[~,motor.fit_sp] = ...
    HelpFun.fittingEDT(x,y,z,PARA,x_eq,y_eq,z_eq,x_ineq,y_ineq,z_ineq);

% Continuous
[~,motor.fit_co] = ...
    HelpFun.fittingEDT(x_co,y_co,z_co,PARA_co,x_eq,y_eq,z_eq);


% Fitting Mechanical Torque Losses
% Calculate inclination
x_1 = T_loss(:,1)./motor.data.n_radps(:,1);
x_1(isnan(x_1)) = [];
x_1 = sum(x_1)/numel(x_1);

% Do fitting (For correct function handle)
motor.fit_lossMech = HelpFun.doLinFit(x_1);

% Fit maximal Power
speed_vec = motor.data.n_radps(:,1);
MaxT = motor.data.p_max_vec_W./speed_vec;
[~,idx] = min(diff(MaxT));

MaxT_vec= MaxT(idx:end);
speed_vec=speed_vec(idx:end);

% Fitting of maximal Power
PARAP.polyX = 2;
PARAP.polyY = 0;
PARAP.Method = 'min SE';

% Do fitting
[fitFun,motor.fit_T_max] = ...
    HelpFun.fittingEDT(speed_vec,zeros(size(speed_vec)),MaxT_vec,PARAP);

%motor.fit_T_max.fitFun = fitFun;

% Evaluation
figure
hold on
plot(motor.data.n_radps(:,1),MaxT)
plot(motor.data.n_radps(:,1),motor.fit_T_max.fitFun(motor.data.n_radps(:,1),zeros(size(motor.data.n_radps(:,1)))))
title('Maximum torque')
xlabel('Motor speed in radps')
ylabel('Motor torque in Nm')
legend('Max torque', 'Fit Max torque in field weakening area')

% Recu limitation
motor.T_recu.FitFun = HelpFun.genFuncHandleTrecu(5, motor.data.n_min_recu_radps, 0, -1*motor.data.T_max_Nm);



cd('Data/PowertrainFitData/Motor')
save('ID3_final','motor');
cd('../../..');


