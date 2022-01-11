function [] = loadVehicleParam(self, v)
% Designed by: Alexander Koch and Tim Bürchner (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Loads vehicle parameters, depending on its inputs
% ------------
% Input:    - self: OptiFrameObject
%              - v: vehicle struct (optimizer form or axle-form)
% ------------

%% Load Default Vehicle Parameters
if nargin ==1 || isempty(v)
    warning('Default vehicle parameters are loaded')
    
    % vehicle mass
    self.veh.m_t =  1.97; % [t]
    
    % rotational mass lambda
    self.veh.lambda = 1.0277; % [-]
    
    % Air Resistance constant
    self.veh.c_a = 0.187;  % [-]
    
    % A_st
    self.veh.a_a = 2.36;  % [m^2]
    
    % Roll Resisitance constant based on J2452_201707
    self.veh.f_r_c0 = 0.0059;   % [-]
    self.veh.f_r_c1 = 6.977964080517644e-05; % [-]
    self.veh.f_r_c2 = 8.970036127410863e-07; % [-]
    
    % Wheel radius
    self.veh.r = 0.351;   % [m]
    
    %Battery
    self.veh.bat.numcellpa = 2;    % [-]
    self.veh.bat.numcelser = 108;  % [-]
    load('Data/PowertrainFitData/Battery/bat_cell_ID3_linear.mat');
    self.veh.bat.bat_cell = bat_cell;
    self.veh.P_max_reku = -50;
    self.veh.P_max_bat = 200;
    
    % Brakes
    self.veh.max_br_trq = -5000;
    
    
    % Motor 1
    if self.indVar == 't'
        load('Data/PowertrainFitData/Motor/ID3_final.mat')
    elseif self.indVar == 's'
        % TODO
    end
    
    self.veh.motor_1 = motor;
    self.veh.motor_1.P_loss_max=max(motor.losses.inverter_W+motor.losses.iron_W+motor.losses.mech_W+motor.losses.stator_W,[],'all')/1000;  % [kW]
    
    % Gear Ratio 1 Motor 1
    self.veh.i_gr1_1 = 11.53;  % [-]
    
    % Gear box 1 Efficiency
    load('Data/PowertrainFitData/Gearbox/GB_ID3_Paper.mat')
    self.veh.gb_1_losses = gearbox;
    self.veh.gb_1_losses.T_loss_max = max(gearbox.losses.T_loss,[],'all');  % [Nm]
    
    if self.twoGears == 1
        % Gear Ratio 2 Motor 1
        self.veh.i_gr1_2 = 5; % [-]
        
        % Additional mass
        self.veh.m_t=self.veh.m_t+0.020;  % [t]
    end
    
    if self.twoMotors ==1
        %Load two smal motors
        
        if self.indVar == 't'
            load('Data/PowertrainFitData/Motor/IM_30.mat')
        elseif self.indVar =='s'
            %TODO
        end
        self.veh.motor_2=motor;
        self.veh.motor_2.P_loss_max=max(motor.losses.inverter_W+motor.losses.iron_W+motor.losses.mech_W+motor.losses.stator_W,[],'all')/1000; % [kW]
        
        % Gear Ratio 1 Motor 2
        self.veh.i_gr2 = 8; % [-]
        
        % Gear box 2 Efficiency
        load('Data/PowertrainFitData/Gearbox/GB_ID3_ASM_Paper.mat')
        self.veh.gb_2_losses = gearbox;
        self.veh.gb_2_losses.T_loss_max = max(gearbox.losses.T_loss,[],'all'); % [Nm]
        
        % Additional mass
        self.veh.m_t=self.veh.m_t+0.080; % [t]
        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load vehicle parameters from vehicle (v) struct
if nargin == 2 && ~isempty(v)
    
    if isfield(v,'axle1')   % First powered axle
        
        % vehicle mass
        self.veh.m_t = v.mass/1000; % [t]
        
        % rotational mass lambda
        self.veh.lambda = v.lambda; % [-]
        
        % Air resistance constant
        self.veh.c_a = v.c_a;       % [-]
        
        % Roll resisitance constant
        self.veh.f_r_c0 = v.wheels.f_r_c0;   % [-]
        self.veh.f_r_c1 = v.wheels.f_r_c1;    % [-]
        self.veh.f_r_c2 = v.wheels.f_r_c2;     % [-]
         
        % Front surface of vehicle
        self.veh.a_a = v.dimensions.a_a; % [m^2]
        
        % Wheel radius
        self.veh.r = v.wheels.r;   % [m]
        
        % Brakes
        self.veh.max_br_trq = v.brakes.T_max_brk;
        
        % % Powertrain
        %Battery
        self.veh.bat.numcellpa = v.battery.cells_parallel;   % [-]
        self.veh.bat.numcelser = v.battery.cells_serial;     % [-]
        self.veh.bat.bat_cell = v.battery.bat_cell;
        
        self.veh.P_max_reku = v.battery.p_max_reku/1000;   % [kW]
        self.veh.P_max_bat = v.battery.p_max_bat/1000;       % [kW]
        
        
        % Motor 1
        self.veh.motor_1= v.axle1.motor; 
        self.veh.motor_1.P_loss_max=max(v.axle1.motor.losses.inverter_W+v.axle1.motor.losses.iron_W+v.axle1.motor.losses.mech_W+v.axle1.motor.losses.stator_W,[],'all')/1000;   % [kW]
        
        % Gearbox
        self.veh.gb_1_losses = v.axle1.gearbox;
        self.veh.gb_1_losses.T_loss_max = max(v.axle1.gearbox.losses.T_loss,[],'all');    % [Nm]
        
        % Gear Ratio 1 Motor 1
        self.veh.i_gr1_1 = v.axle1.gearbox.ratio1;     % [-]
        
        if isfield(v.axle1.gearbox,'ratio2')
            self.veh.i_gr1_2 = v.axle1.gearbox.ratio2;   % [-]
        end
        
        % Motor 2
        
        if isfield( v,'axle2')     %second powered axle
            
            % Motor 2
            self.veh.motor_2= v.axle2.motor;
            self.veh.motor_2.P_loss_max=max(v.axle2.motor.losses.inverter_W+v.axle2.motor.losses.iron_W+v.axle2.motor.losses.mech_W+v.axle2.motor.losses.stator_W,[],'all')/1000;   % [kW]
            
            % Gearbox
            self.veh.gb_2_losses = v.axle2.gearbox;
            self.veh.gb_2_losses.T_loss_max = max(v.axle2.gearbox.losses.T_loss,[],'all');     % [Nm]
            
            
            % Gear Ratio 1 Motor 1
            self.veh.i_gr2 = v.axle2.gearbox.ratio1;     % [-]
        end
        
    else  % vehicle in optimizer form
        veh = v;
    end
    
end

