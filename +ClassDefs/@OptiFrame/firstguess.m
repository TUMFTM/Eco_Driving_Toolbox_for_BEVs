function [] = firstguess (self, v,counter)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Creates a first guess for the optimizer
% ------------
% Input:    - self: OptiFrameObject
%              - v: speed profile of first guess
%              - counter: for relaxed optimization
% ------------
if nargin == 2
    counter =1;
end

% Driving Forces
F_a = 0.5 * self.veh.c_a * self.veh.a_a * self.constants.roh_a .* v(1:end-1).^2;
F_r = (self.veh.f_r_c0+self.veh.f_r_c1*v(1:end-1)+self.veh.f_r_c2*v(1:end-1).^2) * self.veh.m_t * 1000 * 9.81;
F_acc = diff(v) ./ self.paramVariable.delta_t_param .* self.veh.m_t * 1000;

% Torque 1
trq_1 = (F_a + F_r + F_acc) * self.veh.r;
trq_1 =  trq_1 ./ self.veh.i_gr1_1;
n_1 = v(1:end-1)/2/pi/self.veh.r*self.veh.i_gr1_1;
if self.splitGbMap ==1
    trq_1_losses_gb = HelpFun.genPolyFun(self.veh.gb_1_losses.fit_sp,n_1*2*pi,abs(trq_1)); % Small error
else
    trq_1_losses_gb = HelpFun.genPolyFun(self.veh.gb_1_losses.fit_co,n_1*2*pi,(trq_1)); % Small error
end
trq_1_losses_mot = HelpFun.genPolyFun(self.veh.motor_1.fit_lossMech,n_1*2*pi,trq_1+trq_1_losses_gb);
trq_1=trq_1+trq_1_losses_gb+trq_1_losses_mot;

% Motor + PE
P_bat_req = n_1*2*pi .* trq_1+HelpFun.genPolyFun(self.veh.motor_1.fit_sp,n_1*2*pi,trq_1);

% Aux
P_bat_req = P_bat_req + self.paramVariable.p_aux_w;

% SOC with P_bat_req as approximation
start_SOC = self.paramVariable.startSOC;
V_cell_ap = HelpFun.genPolyFun(self.veh.bat.bat_cell.V_fit_bat, start_SOC/100,0);
I_cell_ap = P_bat_req / V_cell_ap/self.veh.bat.numcelser/self.veh.bat.numcellpa;
Q_cell = I_cell_ap .* self.paramVariable.delta_t_param;
SOC_ap =start_SOC - [0; cumsum( 100*Q_cell./(self.veh.bat.bat_cell.Ah*60*60))];

V_cell =  HelpFun.genPolyFun(self.veh.bat.bat_cell.V_fit_bat,SOC_ap/100,zeros(size(SOC_ap)));

% Battery
I_cell = P_bat_req ./(V_cell(1:end-1)*self.veh.bat.numcelser)/self.veh.bat.numcellpa;
P_bat = P_bat_req + (I_cell.^2 * self.veh.bat.bat_cell.R_i_bat) * self.veh.bat.numcellpa * self.veh.bat.numcelser;

start_s = self.ubx(3*self.n+1*self.m+1);

% x0 for 1M1G
x0_T = [trq_1]    ;
x0_b =  zeros(size(x0_T));
x0_v = v;
x0_a = diff(x0_v) ./ self.paramVariable.delta_t_param;
x0_s = start_s+cumtrapz(x0_v .* [self.paramVariable.delta_t_param; self.paramVariable.delta_t_param(end)])       ;
x0_P_b = P_bat / 1000;
x0_I_b = I_cell;
x0_SOC = SOC_ap;
if self.splitMotMap ==1
    x0_Slack = HelpFun.genPolyFun(self.veh.motor_1.fit_sp,n_1*2*pi,x0_T)./1000;
else
    x0_Slack = [];
end
if  self.splitGbMap ==1
    x0_SlackGB = trq_1_losses_gb;
else
    x0_SlackGB = [];
end

x0 = [x0_T; x0_b; x0_a; x0_v; x0_s; x0_P_b; x0_SOC;x0_I_b; x0_Slack; x0_SlackGB] ;


if self.twoGears==1
    
    x0_Cl_rel = 2 .* ones(self.nGear,1);
    trq_1 = (F_a + F_r + F_acc) * self.veh.r;
    trq_1 =  trq_1 ./ self.veh.i_gr1_2;
    n_1_2 = v(1:end-1)/2/pi/self.veh.r*self.veh.i_gr1_2;
    
    
    if self.splitGbMap ==1
        trq_1_losses_gb = HelpFun.genPolyFun(self.veh.gb_1_losses.fit_sp,n_1_2*2*pi,abs(trq_1)); % Small error
    else
        trq_1_losses_gb = HelpFun.genPolyFun(self.veh.gb_1_losses.fit_co,n_1_2*2*pi,(trq_1)); % Small error
    end
    
    
    trq_1_losses_mot = HelpFun.genPolyFun(self.veh.motor_1.fit_lossMech,n_1_2*2*pi,trq_1+trq_1_losses_gb);
    trq_1=trq_1+trq_1_losses_gb+trq_1_losses_mot;
    

    
    if self.splitMotMap ==1
        x0_Slack =HelpFun.genPolyFun(self.veh.motor_1.fit_sp,n_1_2*2*pi,trq_1)./1000; %could be more updated P...
    else
        x0_Slack = [];
    end
    
    if  self.splitGbMap ==1
        x0_SlackGB = trq_1_losses_gb;
    else
        x0_SlackGB = [];
    end
    
    x0_rel = [trq_1; x0_b; x0_a; x0_v; x0_s; x0_P_b; x0_SOC;x0_I_b; x0_Cl_rel; x0_Slack; x0_SlackGB;];
    try
        self.paramVariable.T_t_2_last_param = self.paramVariable.T_t_last_param;  %% could be improved with clutch position
        self.paramVariable.T_b_2_last_param  = self.paramVariable.T_b_last_param;
    end
    
    % Optimize relaxed problem
    [resRel,self.timeRel(counter)] = self.optimizerRelaxed.(strcat("optimiseProfile",self.indVar))(self.paramVariable, self.lbx, self.ubx,x0_rel);
    res = resRel;
    
    
    x0_T_1 = res.T_1;
    x0_T_b = res.T_b;
    x0_a = res.a;
    x0_v = res.v;
    x0_s = res.s;
    x0_P_b = res.P_b;
    x0_SOC = res.SOC;
    x0_I_b = res.I_b;
    
    
    
    x0_Cl = floor(res.Cl+0.1);
    
    x0= [x0_T_1;  x0_T_b; x0_a; x0_v; x0_s; x0_P_b; x0_SOC;x0_I_b; x0_Cl; x0_Slack; x0_SlackGB];
    
    
end

if self.twoMotors==1
    % Splitt trq_1
    x0_T = 0.5.* x0_T;
    x0 = [x0_T; x0_b; x0_a; x0_v; x0_s; x0_P_b; x0_SOC;x0_I_b; x0_Slack; x0_SlackGB];
    
    
    trq_3 = 0.5*(F_a + F_r + F_acc) * self.veh.r;
    trq_3 =  trq_3 ./ self.veh.i_gr2;
    n_2 = v(1:end-1)/2/pi/self.veh.r*self.veh.i_gr2;
    
    if self.splitGbMap ==1
        trq_3_losses_gb = HelpFun.genPolyFun(self.veh.gb_2_losses.fit_sp,n_2*2*pi,abs(trq_3)); % Small error
    else
        trq_3_losses_gb = HelpFun.genPolyFun(self.veh.gb_2_losses.fit_co,n_2*2*pi,(trq_3)); % Small error
    end
    
    
    trq_3_losses_mot = HelpFun.genPolyFun(self.veh.motor_2.fit_lossMech,n_2*2*pi,trq_3+trq_3_losses_gb);
    trq_3=trq_3+trq_3_losses_gb+trq_3_losses_mot;
    
    if self.splitMotMap ==1
        x0_Slack_2 = HelpFun.genPolyFun(self.veh.motor_2.fit_sp,n_2*2*pi,trq_3)./1000;
    else
        x0_Slack_2 = [];
    end
    if self.splitGbMap ==1
        x0_SlackGB2=trq_3_losses_gb;
    else
        x0_SlackGB2 = [];
    end
    
    x0 = [x0; trq_3; x0_Slack_2; x0_SlackGB2];
    
end

self.x0=x0;
end