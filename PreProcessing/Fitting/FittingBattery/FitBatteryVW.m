% Skript to fit battery of VW ID3
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2021 
% ------------
% Version: Matlab2021a
%-------------

%Path
addpath(genpath('PreProcessing/Fitting/FittingTool'));


%Read Data
M = readmatrix('DataExtern/PowertrainComponents/Battery/ID3Battery.csv'); % based on Wassiliadis et. al - Quantifying the state of the art of electric powertrains in battery electric vehicles:
%Range, power, and lifetime from component to system level of the VW ID.3

OCV = M(:,1);
SOC = M(:,2);

% Cut relevant SOC area
OCV_relevant = OCV((SOC>0.041));
SOC_relevant = SOC(SOC>0.041);
OCV_relevant =  OCV_relevant(SOC_relevant<=0.97);
SOC_relevant = SOC_relevant(SOC_relevant<=0.97);

% Data for Fitting
xM_use = SOC_relevant;
MAPM_use = OCV_relevant;
yM_use = zeros(size(xM_use));

x_eq = SOC_relevant(1);
y_eq = 0;
z_eq = OCV_relevant(1);

% Options for Fitting
PARA.Method = 'min SRE';
PARA.polyX = 1;
PARA.polyY = 0;

% Fitting
[fitFun,fit_bat] = ...
    HelpFun.fittingEDT(xM_use,yM_use,MAPM_use,PARA,x_eq,y_eq,z_eq);

% Evaluation Fit
figure
plot(SOC_relevant,OCV_relevant)
hold on
plot(SOC_relevant,fitFun(SOC_relevant,zeros(size(SOC_relevant))))
legend('Data','Fit')
xlabel('SOC')
ylabel('Voltage')

% Generate bat_cell struct
bat_cell.V_fit_bat = fit_bat;
bat_cell.fitFun = fitFun;
bat_cell.R_i_bat = 0.001857;
bat_cell.Ah =  80.44;

bat_cell.OCV = OCV;
bat_cell.SOC = SOC;
bat_cell.V_max = OCV(1);

% Save
cd('Data/PowertrainFitData/Battery')
save('bat_cell_ID3_linear.mat','bat_cell');
cd('../../..');


