function [] = defineBoundsx(self)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Defines the bounds of the state and control vector x
% ------------
% Input:    - self: OptiFrameMovingHorizon Object
% ------------

%Interface for easier readability
m = self.m;
twoGears=self.twoGears;
twoMotors=self.twoMotors;
veh = self.veh;
paramConstant = self.paramConstant;
nGear = self.nGear;
splitGbMap = self.splitGbMap;
splitMotMap = self.splitMotMap;
est=self.est;
cs = self.cs;
lbx = [];
ubx = [];

% Bounds for motor torque
boundsx.lb_T_1 = [ -ones(m-1,1) * veh.motor_1.data.T_max_Nm] ;
boundsx.ub_T_1 = [ ones(m-1,1) * veh.motor_1.data.T_max_Nm];
lbx=[lbx; boundsx.lb_T_1];
ubx=[ubx; boundsx.ub_T_1];

% Braking Torque
boundsx.lb_T_b = [ ones(m-1,1) * veh.max_br_trq] ;
boundsx.ub_T_b = [ zeros(m-1,1)];
lbx=[lbx; boundsx.lb_T_b];
ubx=[ubx; boundsx.ub_T_b];


% Bounds for acceleration:  initial acceleration - other acceleration
boundsx.lb_a = [ ones(m-1,1) * paramConstant.a_max_b] ;
boundsx.ub_a = [ones(m-1,1) * paramConstant.a_max_t];
lbx=[lbx; boundsx.lb_a];
ubx=[ubx; boundsx.ub_a];



% initial velocity - other velocities - final velocity

boundsx.lb_v = [ cs.v; zeros(m-1,1)] ;
boundsx.ub_v =  [cs.v; max(self.dm.v_max)*ones(m-1,1)] ;
lbx=[lbx; boundsx.lb_v];
ubx=[ubx; boundsx.ub_v];


% initial position - other positions - final position
boundsx.lb_s = [cs.s * ones(m,1)] ;
boundsx.ub_s =  [cs.s; est.s(2:end)] ;
lbx=[lbx; boundsx.lb_s];
ubx=[ubx; boundsx.ub_s];


% Power Battery
boundsx.lb_Pbat = [ veh.P_max_reku.*ones(m-1,1)] ;
boundsx.ub_Pbat =  [ veh.P_max_bat.*ones(m-1,1)] ;
lbx=[lbx; boundsx.lb_Pbat];
ubx=[ubx; boundsx.ub_Pbat];


% SOC Battery
boundsx.lb_SOC = [zeros(m,1)] ;
boundsx.ub_SOC =  [ 100*ones(m,1)] ;
lbx=[lbx; boundsx.lb_SOC];
ubx=[ubx; boundsx.ub_SOC];

% Current Battery
boundsx.lb_Ibat = [ -inf*ones(m-1,1)] ;
boundsx.ub_Ibat =  [ inf.*ones(m-1,1)] ;
lbx=[lbx; boundsx.lb_Ibat];
ubx=[ubx; boundsx.ub_Ibat];




% Bounds GearLeaver
if twoGears ==1
    boundsx.lb_Cl = [ones(nGear,1)] ;
    boundsx.ub_Cl =  [ 2*ones(nGear,1)] ;
    lbx=[lbx; boundsx.lb_Cl];
    ubx=[ubx; boundsx.ub_Cl];
end

% bounds torque motor 2
if twoMotors ==1
    
    boundsx.lb_T_2 = [ -ones(m-1,1) * veh.motor_2.data.T_max_Nm] ;
    boundsx.ub_T_2 = [ ones(m-1,1) * veh.motor_2.data.T_max_Nm];
    lbx=[lbx; boundsx.lb_T_2];
    ubx=[ubx; boundsx.ub_T_2];
    
end

%% Slacks

if splitMotMap == 1
    % Slack for Motor losses motor 1
    boundsx.lb_Sl_mot = [ zeros(m-1,1)] ;
    boundsx.ub_Sl_mot = [ veh.motor_1.P_loss_max*ones(m-1,1)];
    lbx=[lbx; boundsx.lb_Sl_mot];
    ubx=[ubx; boundsx.ub_Sl_mot];
end

% Slack for Gearbox losses GB1
if splitGbMap == 1
    boundsx.lb_Sl_gb = [ zeros(m-1,1)] ;
    boundsx.ub_Sl_gb = [ veh.gb_1_losses.T_loss_max*ones(m-1,1)];
    lbx=[lbx; boundsx.lb_Sl_gb];
    ubx=[ubx; boundsx.ub_Sl_gb];
    
end

if twoMotors ==1
    % Slack for Motor losses motor 1
    if splitMotMap ==1
        
        boundsx.lb_Sl_mot_2 = [ zeros(m-1,1)] ;
        boundsx.ub_Sl_mot_2 = [ veh.motor_2.P_loss_max*ones(m-1,1)];
        lbx=[lbx; boundsx.lb_Sl_mot_2];
        ubx=[ubx; boundsx.ub_Sl_mot_2];
        
    end
    if splitGbMap ==1
        % Slack for Gearbox losses GB1
       
        boundsx.lb_Sl_gb_2 = [ zeros(m-1,1)] ;
        boundsx.ub_Sl_gb_2 = [ veh.gb_2_losses.T_loss_max*ones(m-1,1)];
        lbx=[lbx; boundsx.lb_Sl_gb_2];
        ubx=[ubx; boundsx.ub_Sl_gb_2];
        
    end
end

self.ubx=ubx;
self.lbx=lbx;
self.boundsx=boundsx;

end

