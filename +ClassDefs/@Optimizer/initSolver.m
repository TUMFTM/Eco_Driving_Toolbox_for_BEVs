function [] = initSolver(self)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Initilizes the optimizer based on the vehicle and
% parameters. Problem definition differs slightly, depending on
% optimization type (Route or MovingHorizon)
% ------------
% Input:    - self: Optimizer Object
% ------------


import casadi.*

if strcmp(self.indVar, 's') % Initilize solver over distance --> Not supported, yet
    initSolverS(self)
    return
end


%% Empty Constraints
g = [];       % constraint vector
lbg = [];    % constraint vector lower bound
ubg = [];   % constraint vector upper bound

%% Optimization Variables
T_1 = SX.sym('T_1', self.n);            % Torque of Motor 1 in Nm
T_b =  SX.sym('T_b', self.n);           % Braking Torque in Nm
a = SX.sym('a', self.n);                    % Acceleration in m/s^2
v = SX.sym('v', self.m);                   % Speed in m/s
s = SX.sym('s', self.m);                    % Distance in m
P_b = SX.sym('P_b', self.n);             % Power in Battery in kW
SOC = SX.sym('SOC', self.m);        % SOC in %
I_b = SX.sym('I_b', self.n);             % Battery Current in A

% Splitted motor map
if self.splitMotMap == 1
    Slack = SX.sym('Slack', self.n);        % Slack that describes losses in motor
else
    Slack = [];
end

% Splitted garbox map
if self.splitGbMap == 1
    Slack_GB = SX.sym('Slack_GB', self.n);        % Slack that describes losses in gearbox
else
    Slack_GB = [];
end

% Two gears
if self.twoGears ==1
    Cl = SX.sym('Cl',self.nGear);      % Gear position (Gear 1 or Gear 2)
else
    Cl = [];
end

% Two motors
if self.twoMotors ==1
    T_2 = SX.sym('T_2', self.n) ;            % Traction Torque motor 2 gear 1
    
    % Splitted motor map
    if  self.splitMotMap == 1
        Slack_2 = SX.sym('Slack_2', self.n);
    else
        Slack_2 = [];
    end
    
    % Splitted garbox map
    if self.splitGbMap == 1
        Slack_GB2 = SX.sym('Slack_GB2', self.n);
    else
        Slack_GB2 = [];
    end
    
else
    T_2 = [];
    Slack_2 = [];
    Slack_GB2=[];
end


%% Complete State Vector
x = [T_1; T_b; a; v; s; P_b; SOC;I_b; Cl; T_2;Slack;Slack_GB;Slack_2;Slack_GB2]; % T2, Cl and Slacks are optional depending on options


%% Online Parameters (depending on Route or MPC)
delta_t_param = SX.sym('delta_t_param', self.n);            % delta t [s]
a_last_param = SX.sym('a_last_param', 1);                       % acceleration of last step in [m/s^2]
delta_t_last_param = SX.sym('delta_t_last_param', 1);       % duration of last step in [s]
p_aux_w = SX.sym('p_aux_w',1);                                          % Auxilary Power in [W]
startSOC = SX.sym('startSOC',1);                                           % StartSOC in [%]
v_max_vec = SX.sym('v_max_vec',  self.numSpeedLim);   % maximal speed in [m/s]
v_max_svec = SX.sym('v_max_svec',  self.numSpeedLim);   % starting distance point of maximal speed in [m]

if self.type == 'R'
    a_next_param = SX.sym('a_next_param', 1);                       % acceleration of next (future) step in [m/s^2]
    delta_t_next_param = SX.sym('delta_t_next_param', 1);       % duration of next (future) step in [s]
    if self.limitOverallRoute == 1
        a_max_sq_sum = SX.sym('a_max_sq_sum', 1); % maximal summed acceleration
        j_max_sq_sum = SX.sym('j_max_sq_sum', 1);   % maximal summed jerk
    else
        a_max_sq_sum = [];
        j_max_sq_sum = [];
    end
else
    a_next_param = [];
    delta_t_next_param = [];
    a_max_sq_sum = [];
    j_max_sq_sum = [];
end

if strcmp(self.type, 'M')
    s_lead_veh = SX.sym('s_lead_veh', self.m); % position leading vehicle  in [m]
    T_1_last_param = SX.sym('T_1_last_param', 1);   % last torque of motor 1 in [Nm]
    T_b_last_param = SX.sym('T_b_last_param', 1);   % last torque of brakes in [Nm]
    if self.twoMotors == 1
        T_2_last_param = SX.sym('T_2_last_param', 1); % last torque of motor 2 in [Nm]
    else
        T_2_last_param = [];
    end
else
    s_lead_veh = [];
    T_1_last_param = [];
    T_b_last_param = [];
    T_2_last_param = [];
end

%% Generate Parametervector
param_vec = vertcat(delta_t_param, ...
    a_last_param, ...
    delta_t_last_param, ...
    p_aux_w,...
    startSOC,...
    v_max_svec,...
    v_max_vec,...
    a_next_param, ...
    delta_t_next_param,...
    a_max_sq_sum,...
    j_max_sq_sum,...
    s_lead_veh,...
    T_1_last_param,...
    T_b_last_param,...
    T_2_last_param);
      
%% Rightsizing of Cl
if self.twoGears == 1
    Cl =  reshape(repmat(Cl,uint8(1),uint8(self.n/self.nGear))',self.n,1);
end

%% Constant Coefficient
self.veh.c_F_a = 0.5 * self.veh.c_a * self.veh.a_a * self.constants.roh_a;

%% Calculation of Motor speed
n_m1 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_1; % Motor 1 speed  in [1/s]

if self.twoGears == 1
    n_m1_2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_2; % Motor 1 speed gear 2 in [1/s]
    n_m1 = (2-Cl) .* n_m1+ (Cl-1) .*  n_m1_2;  % Actual Motor speed as function of clutch position
end

if self.twoMotors==1
    n_m2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr2;
else
    n_m2 = [];
end


%% Mechanical Part: Calculation of Acceleration Force
[f_acc,~,~,~,~,T_loss_gb_1_a,T_loss_gb_1_b,T_loss_gb_2_a,T_loss_gb_2_b] = calcAccForce(self, v, T_1, T_b, T_2, n_m1,n_m2, Cl,Slack_GB,Slack_GB2);

if self.splitGbMap == 1
    % Constraint SlackGB
    sl_cst_1 = Slack_GB-T_loss_gb_1_a;
    g = [g; sl_cst_1];
    ubg = [ubg; inf(self.n,1)];
    lbg = [lbg; zeros(self.n,1)];
    
    % Constraint SlackGB
    sl_cst_2 = Slack_GB-T_loss_gb_1_b;
    g = [g; sl_cst_2];
    ubg = [ubg; inf(self.n,1)];
    lbg = [lbg; zeros(self.n,1)];
    
    if self.twoMotors == 1
        % Constraint SlackGB
        sl_cst_1 = Slack_GB2-T_loss_gb_2_a;
        g = [g; sl_cst_1];
        ubg = [ubg; inf(self.n,1)];
        lbg = [lbg; zeros(self.n,1)];
        
        % Constraint SlackGB
        sl_cst_2 = Slack_GB2-T_loss_gb_2_b;
        g = [g; sl_cst_2];
        ubg = [ubg; inf(self.n,1)];
        lbg = [lbg; zeros(self.n,1)];
    end
end



%% Electrical Part: Calculation of requried Electrical Power
% Motor + PE 1
[P_el_1, ubg,lbg,g] = integrate_motor(self.veh.motor_1, n_m1, T_1, Slack, self.splitMotMap, ubg,lbg,g,self.n);


% Motor + PE 2
if ~isempty(n_m2)
    [P_el_2, ubg,lbg,g] = integrate_motor(self.veh.motor_2, n_m2, T_2, Slack_2, self.splitMotMap, ubg,lbg,g,self.n);
else
    P_el_2 = [] ;
end

% Battery
P_b_req = P_el_1 + p_aux_w/1000;
if ~isempty(n_m2)
    P_b_req = P_b_req + P_el_2;
end

% Number of cells in vehicle
num_cell = self.veh.bat.numcelser * self.veh.bat.numcellpa;

% Battery calculations
V_cell = HelpFun.genPolyFun(self.veh.bat.bat_cell.V_fit_bat,SOC/100,zeros(size(SOC)));  
P_b_loss = I_b.^2 * (self.veh.bat.bat_cell.R_i_bat) *num_cell/1000;

P_b2 = I_b.*V_cell(1:end-1)*num_cell/1000;

% Maximal Recuperation
V_max = self.veh.bat.bat_cell.V_max;
delta_V_charge =  V_max-V_cell(1:end-1);
I_charge = delta_V_charge/self.veh.bat.bat_cell.R_i_bat;
P_charge_max = -1* I_charge * V_max * num_cell/1000;   % Hier liegt ein Problem
Q_cell = I_b .* delta_t_param;

% Battery constraint I
pb1_cst =P_b2-P_b_loss-P_b_req;
g = [g; pb1_cst];
ubg = [ubg; zeros(self.n,1)];
lbg = [lbg; zeros(self.n,1)];

% Battery constraint II
g=[g; P_b - P_b2];
ubg =  [ubg; zeros(self.n,1)];
lbg =   [lbg; zeros(self.n,1)];

% Constraint battery - maximal charging --> faster with this one better
% with next one
g=[g; P_charge_max - (P_b)];
ubg =  [ubg; zeros(self.n,1)];
lbg =   [lbg; -inf(self.n,1)];

% % % Constraint battery - maximal charging
g=[g; (-I_charge-I_b)*100];
ubg =  [ubg; zeros(self.n,1)];
lbg =   [lbg; -inf(self.n,1)];

% Constraint battery - SOC-Start
SOCstart_cst = startSOC-SOC(1);
g=[g; SOCstart_cst];
ubg =  [ubg; zeros(1,1)];
lbg =   [lbg; zeros(1,1)];

% Constraint battery - SOC
SOC_cst = diff(SOC) +  100*Q_cell./(self.veh.bat.bat_cell.Ah*60*60);
g=[g; SOC_cst];
ubg =  [ubg; zeros(self.n,1)];
lbg =   [lbg; zeros(self.n,1)];


%% General Constraints
% Constraint: forces <-> acceleration
a_cst = a - (f_acc ./ (self.veh.m_t*self.veh.lambda));
g = [g; a_cst];
ubg = [ubg; zeros(self.n,1)];
lbg = [lbg; zeros(self.n,1)];

% Constraint: acceleration <-> velocity
v_cst = -diff(v) + a .* delta_t_param;
g = [g; v_cst];
ubg = [ubg; zeros(self.n,1)];
lbg = [lbg; zeros(self.n,1)];

% Constraint: velocity <-> position
s_cst = -diff(s) + 0.5 .* (v(1:end-1) + v(2:end)) .* delta_t_param;
g = [g; s_cst];
ubg = [ubg; zeros(self.n,1)];
lbg = [lbg; zeros(self.n,1)];

% Max velocity
v_max= determine_vmax(v_max_svec,v_max_vec,s);
v_max_cst = v_max-v;
g = [g; v_max_cst];
ubg = [ubg; inf(self.m,1)];
lbg = [lbg; zeros(self.m,1)];


%% Driving Style Constraints
% Constraint: jerk
if self.type == 'R'
    delta_t_jerk = [delta_t_last_param; delta_t_param; delta_t_next_param];
    delta_t_jerk =  0.5 .* (delta_t_jerk(1:end-1) + delta_t_jerk(2:end));
    j_cst = diff([a_last_param; a; a_next_param]) ./ delta_t_jerk;
    g = [g; j_cst];
    ubg = [ubg; self.paramConstant.j_max * ones(self.n+1,1)];
    lbg = [lbg; -self.paramConstant.j_max * ones(self.n+1,1)];
elseif strcmp(self.type,'M')
    delta_t_jerk = [delta_t_last_param; delta_t_param];
    delta_t_jerk =  0.5 .* (delta_t_jerk(1:end-1) + delta_t_jerk(2:end));
    j_cst = diff([a_last_param; a]) ./ delta_t_jerk;
    g = [g; j_cst];
    ubg = [ubg; self.paramConstant.j_max * ones(self.n,1)];
    lbg = [lbg; -self.paramConstant.j_max * ones(self.n,1)];
end

% Constraint: acceleration (additional Constraint in a is faster than lbx
% ubx only!)
a_lb_cst = a;
g = [g; a_lb_cst];
ubg = [ubg; self.paramConstant.a_max_t * ones(self.n,1)];
lbg = [lbg; self.paramConstant.a_max_b * ones(self.n,1)];

%% Powertrain Constraints
%Motor 1 Power (over torque)
T_max =  HelpFun.genPolyFun(self.veh.motor_1.fit_T_max,n_m1.*2 .* pi,zeros(size(n_m1)));

p_m1_t_cst = T_max-T_1; % [kW]
g=[g; p_m1_t_cst];
ubg =  [ubg; inf(self.n,1)];
lbg =   [lbg; zeros(self.n,1)];

p_m1_t_cst = -T_max-T_1; % [kW]
g=[g; p_m1_t_cst];
ubg =  [ubg; zeros(self.n,1)];
lbg =   [lbg; -inf(self.n,1)];


% Motor 1 Speed
n_m1_cst = n_m1 - self.veh.motor_1.data.n_max_rpm./60;
g = [g; n_m1_cst];
ubg = [ubg; zeros(self.n,1)];
lbg = [lbg; -inf(self.n,1)];

% %Motor 1 Regenerative braking constraint
if isfield(self.veh.motor_1,'T_recu') && self.minTorqueRecu == 1
    T_min = self.veh.motor_1.T_recu.FitFun(n_m1*2*pi);
    T_recu_cst = T_1 - T_min;
    g = [g; T_recu_cst];
    ubg =  [ubg; inf .* ones(self.n,1)];
    lbg =   [lbg; zeros(self.n,1)];
end


if self.twoMotors == 1
%Motor 2 Power over torque
T_max2 =  HelpFun.genPolyFun(self.veh.motor_2.fit_T_max,n_m2.*2 .* pi,zeros(size(n_m1)));

p_m2_t_cst = T_max2-T_2; % [kW]
g=[g; p_m2_t_cst];
ubg =  [ubg; inf(self.n,1)];
lbg =   [lbg; zeros(self.n,1)];

p_m2_t_cst = -T_max2-T_2; % [kW]
g=[g; p_m2_t_cst];
ubg =  [ubg; zeros(self.n,1)];
lbg =   [lbg; -inf(self.n,1)];


% %Motor 2 Regenerative braking constraint
if isfield(self.veh.motor_2,'T_recu') && self.minTorqueRecu == 1
    T_min = self.veh.motor_2.T_recu.FitFun(n_m2*2*pi);
    T_recu_cst = T_2 - T_min;
    g = [g; T_recu_cst];
    ubg =  [ubg; inf .* ones(self.n,1)];
    lbg =   [lbg; zeros(self.n,1)];
end

    
%     
    % Motor 2 Speed
    n_m2_cst = n_m2 - self.veh.motor_2.data.n_max_rpm./60;
    g = [g; n_m2_cst];
    ubg = [ubg; zeros(self.n,1)];
    lbg = [lbg; -inf(self.n,1)];
end



%% Route Optimization Constraints
if strcmp(self.type , 'R') &&  self.limitOverallRoute == 1
    %Constraint for Route Optimization
    com1_cst = sum1((a).^2)-a_max_sq_sum;
    g = [g; com1_cst];
    ubg = [ubg; 0];
    lbg = [lbg; -inf];
    
    com2_cst = sum1(j_cst.^2)-j_max_sq_sum;
    g = [g; com2_cst];
    ubg = [ubg; 0];
    lbg = [lbg; -inf];
end


% Inertial Work Rating (Cycle Beating)
if strcmp(self.type , 'R') &&  self.limitIWR == 1
    ds_IWR = diff(s);
    
    
    %stepFun =  0.5+atan(100*a)/pi;
    %heVar = max(a,0);
    
    IWR_d = (a .* ds_IWR);
    IWR_d = max(0,IWR_d);
    IWR_driven = sum(IWR_d);
    %IWR_target = sum(self.paramConstant.refCycle.a(self.paramConstant.refCycle.a>0) .*self.paramConstant.refCycle.ds(self.paramConstant.refCycle.a>0));
    
    IWR_target = (self.paramConstant.refCycle.a .*self.paramConstant.refCycle.ds);
    IWR_target(IWR_target<0) = 0;
    IWR_target = sum(IWR_target);
    
    IWR_cst = (IWR_driven-IWR_target)/IWR_target;
    g = [g; IWR_cst];
    ubg = [ubg; self.paramConstant.ubIWR];
    lbg = [lbg; self.paramConstant.lbIWR];  
end


% RMSSE velocity (Cycle Beating)
if strcmp(self.type , 'R') &&  self.limitRMSSEv == 1
    
    RMSSEvsqared = sum((v-self.paramConstant.refCycle.v).^2) / length(v);

    g = [g; RMSSEvsqared];
    ubg = [ubg; self.paramConstant.ubRMSSEv^2];
    lbg = [lbg; self.paramConstant.lbRMSSEv^2];  
end






%% MPC Optimization Constraints
if strcmp(self.type , 'M')
  
    int_veh_dist = s_lead_veh - s;  % Distance to Leading vehicle
    
    % Constraint Minimum Distance to leading vehicle
    s_lvl_cst = int_veh_dist - self.paramConstant.t_dmin * v - self.paramConstant.s_min_v0;
    g = [g; s_lvl_cst];
    ubg = [ubg; inf(self.m,1)];
    lbg = [lbg; zeros(self.m,1)];
    
    % Constraint Maximum Distance to leading vehicle
    if self.maxDistance == 1
        s_lvu_cst = self.paramConstant.s_max_v0+ (self.paramConstant.t_dmax * v) - int_veh_dist;
        g = [g; s_lvu_cst];
        ubg = [ubg; inf(self.m,1)];
        lbg = [lbg; zeros(self.m,1)];
    end
else
    int_veh_dist = [];
end


%% Cost Function
J = calcCostFun(self, j_cst, a, v,s,T_1,T_2,T_b, P_b,...
     delta_t_jerk, delta_t_param,...
     T_1_last_param,T_2_last_param, T_b_last_param,delta_t_last_param,n_m2,...
     int_veh_dist,v_max);


%% Define Solver Options
% Define nlp
nlp = struct('x', x, 'p', param_vec, 'f', J, 'g', g);

% Define options
if self.type == 'R'
    opts_IPOPT = struct('expand', false, ...
        'verbose', false);
elseif strcmp(self.type , 'M')
    opts_IPOPT = struct('expand', false, ...
        'verbose', false,...
        'print_time', 0);
end

if self.type == 'R'
    opts_IPOPT.ipopt = struct(...
        'print_level', 6,...
        'max_iter', 10000,...
        'tol', 1e-4,...
        'nlp_scaling_method', 'none');
elseif strcmp(self.type , 'M')
    opts_IPOPT.ipopt = struct(...
        'print_level', 0,...
        'max_iter', 1500,...
        'tol', 1e-4,...
        'nlp_scaling_method', 'none');
end

% Solver
self.solver = nlpsol('solver', 'ipopt', nlp, opts_IPOPT);

% Save Vectors
self.x = x;
self.g = g;
self.lbg = lbg;
self.ubg = ubg;
self.J = J;

disp('Solver is initialized!')
end



function [J] = calcCostFun(self,jerk, a, v,s,T_1,T_2,T_b, P_b, delta_t_jerk,...
    delta_t_param,T_1_last_param,T_2_last_param, T_b_last_param,delta_t_last_param,n_m2,int_veh_dist,v_max)
%CALC_COSTFUN Calculates costs for optimization problem

%% Preprocessing
% Prepare Scaling Factor for w_a and w_j in MPC for smoother speed
% profiles
if strcmp(self.type , 'M')
    if self.scalingwjwa == 1
        scaling_f = max(v(1)^2,1);
    else
        scaling_f = 1;
    end
elseif self.type == 'R'
    scaling_f = 1;
end

% Prepare Regularization Vectors
if self.type == 'R'
    r_T_1 = (diff(T_1) ./ delta_t_param(1:end-1)).^2.*delta_t_param(1:end-1);
    r_T_b = (diff(T_b) ./ delta_t_param(1:end-1)).^2.*delta_t_param(1:end-1);
    if ~isempty(n_m2)
        r_T_2 = (diff(T_2) ./ delta_t_param(1:end-1)).^2.*delta_t_param(1:end-1);
    end
elseif strcmp(self.type , 'M')
    r_T_1 = (diff([T_1_last_param; T_1]) ./ [delta_t_last_param; delta_t_param(1:end-1)]).^2 .*[delta_t_last_param; delta_t_param(1:end-1)];
    r_T_b = (diff([T_b_last_param; T_b]) ./ [delta_t_last_param; delta_t_param(1:end-1)]).^2 .*[delta_t_last_param; delta_t_param(1:end-1)];
    if ~isempty(n_m2)
        r_T_2 = (diff([T_2_last_param; T_2]) ./ [delta_t_last_param; delta_t_param(1:end-1)]).^2 .*[delta_t_last_param; delta_t_param(1:end-1)];
    end
end



%% Cost Function
J =   ...
     self.paramWeighting.w_j * sum(jerk.^2 .* delta_t_jerk) * scaling_f ...
    + self.paramWeighting.w_a * sum1(a.^2  .* delta_t_param)  * scaling_f ...
    + self.paramWeighting.w_E * sum1(P_b .* delta_t_param)  ... 
    + self.paramWeighting.w_r * sum1(r_T_1) ...
    + self.paramWeighting.w_r_br * sum1(r_T_b) ...
    - self.paramWeighting.w_v * sum1(v(1:end-1).* delta_t_param) ...
    + self.paramWeighting.w_v_lim * sum1((v_max(1:end-1)-v(1:end-1)).^2.* delta_t_param)...
    -  self.paramWeighting.w_vEnd * (0.5 * (self.veh.m_t*self.veh.lambda) * (v(end)^2-v(1)^2)) ...
    + self.paramWeighting.w_cons  * sum1(P_b .* delta_t_param) / (s(end)-s(1)+0.0001);

if strcmp(self.type , 'M')
    J = J + self.paramWeighting.w_s * sum1((int_veh_dist-self.paramConstant.s_max_v0-(v*self.paramConstant.t_ds)).^2 .* [delta_t_jerk; delta_t_jerk(end)] );
end

if ~isempty(n_m2)
    J = J + self.paramWeighting.w_r * sum1(r_T_2);
end

end


function [P_el, ubg,lbg,g] = integrate_motor(motor, n_m, T, Slack,splitMotMap,ubg,lbg,g,n)

if splitMotMap == 1
    P_el_loss_p = calc_motor_power_loss_sp(motor,n_m.*2.*pi,T);
    P_el_loss_n = calc_motor_power_loss_sp(motor,n_m.*2.*pi,-T);
    
    % Constraint Slack
    sl_cst_1 = Slack-P_el_loss_p;
    g = [g; sl_cst_1];
    ubg = [ubg; inf(n,1)];
    lbg = [lbg; zeros(n,1)];
    
    % Constraint Slack
    sl_cst_2 = Slack-P_el_loss_n;
    g = [g; sl_cst_2];
    ubg = [ubg; inf(n,1)];
    lbg = [lbg; zeros(n,1)];
    
    P_el_1_loss = Slack;
else
    P_el_1_loss = calc_motor_power_loss_co(motor,n_m.*2.*pi,T);
    
end

P_mot_mech = T.*n_m.*2.*pi/1000;


P_el = P_el_1_loss+P_mot_mech;
end

function [power] = calc_motor_power_loss_sp(motor,omega,T)

[p_l_m] = HelpFun.genPolyFun(motor.fit_sp,omega,T);
power  =  (p_l_m) / 1000 ;

end

function [power] = calc_motor_power_loss_co(motor,omega,T)

[p_l_m] = HelpFun.genPolyFun(motor.fit_co,omega,T);
power  =  (p_l_m) / 1000 ;

end


function funVmax =  determine_vmax(vec_s,vmax,s)

scaling = 150;
 funVmax = vmax(1);

for i = 2:length(vmax)
    
    v_change = vmax(i)-vmax(i-1);
     s_change = vec_s(i);
    funVmax = funVmax + ...
         v_change *  (0.5+  atan(scaling*(s-s_change))/pi);
end
end












