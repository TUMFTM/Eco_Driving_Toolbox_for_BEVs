function [f_acc,T_loss_m_1,T_loss_m_2,T_loss_gb_1,T_loss_gb_2,T_loss_gb_1_a,T_loss_gb_1_b,T_loss_gb_2_a,T_loss_gb_2_b] = calcAccForce(self, v, T_1, T_b, T_2, n_1,n_2, Cl,Slack_GB,Slack_GB2)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Calculates the acceleration force based on motor torque,
% braking torque and selected gear. Furthermore gearbox losses and mechanical motor losses are
% calculated. The calculation differ, depending on the option
% splitGBMap.
% ------------
% Input:    - self: Optimizer Object
%              - v:  vector [m x 1], velocity of vehicle in [m/s]
%              - T_1:  vector [n x 1], Motor air gap torque of motor 1
%              in [Nm]
%              - T_b:  vector [n x 1], Braking torque in [Nm]
%              - T_2:  vector [n x 1], Motor air gap torque of motor 2
%              in [Nm] 
%              - n_1:  vector [n x 1], Motor speed of motor 1 in [1/s]
%              - n_2:  vector [n x 1], Motor speed of motor 2 in [1/s]
%              - Cl:   vector [nGear x 1], Clutch position (Gear 1 or 
%              2) in []
%              - Slack_GB:  vector [n x 1], Optimization Variabe: Losses of gearbox of motor 1 in [Nm]
%              - Slack_GB2:  vector [n x 1], Optimization Variabe: Losses of gearbox of motor 2 in [Nm]
% ------------
% Output: - f_acc: vector [n x 1] acceleration force in [kN]
%              - T_loss_m_1:  vector [n x 1], Mechanical motor losses of motor 1 in [Nm]
%              - T_loss_m_2:  vector [n x 1], Mechanical motor losses of motor 2 in [Nm]
%              - T_loss_gb_1:  vector [n x 1],  Gearbox losses of gearbox 1 in [Nm]
%              - T_loss_gb_2:  vector [n x 1], Gearbox losses of gearbox 2 in [Nm]
%              - T_loss_gb_1_a:  vector [n x 1], Gearbox losses of gearbox 1 for positive torques with split map in [Nm]
%              - T_loss_gb_1_b:  vector [n x 1], Gearbox losses of gearbox 1 for negative torques with split map in [Nm]
%              - T_loss_gb_2_a:   vector [n x 1], Gearbox losses of gearbox 2 for positive torques with split map in [Nm]
%              - T_loss_gb_2_b:  vector [n x 1], Gearbox losses of gearbox 2 for negative torques with split map in [Nm]
% ------------





% Mechanical  Motor Losses
T_loss_m_1 = HelpFun.genPolyFun(self.veh.motor_1.fit_lossMech,n_1*2*pi,T_1);

% GB losses
if self.splitGbMap == 1 % Generate two T losses depending on sign of T_in
    T_loss_gb_1_a = HelpFun.genPolyFun(self.veh.gb_1_losses.fit_sp,n_1*2*pi,(T_1-T_loss_m_1));
    T_loss_gb_1_b = HelpFun.genPolyFun(self.veh.gb_1_losses.fit_sp,n_1*2*pi,-(T_1-T_loss_m_1));
    T_loss_gb_1 = Slack_GB;
else
    T_loss_gb_1_a = [];
    T_loss_gb_1_b = [];
    T_loss_gb_1 = HelpFun.genPolyFun(self.veh.gb_1_losses.fit_co,n_1*2*pi,(T_1-T_loss_m_1));
end


%% Prepare f_acc
f_acc = -self.veh.c_F_a .* v(1:end-1).^2 * 0.001 ...
    - (self.veh.f_r_c0 + self.veh.f_r_c1 * v(1:end-1)+self.veh.f_r_c2 *v(1:end-1).^2) * self.constants.g_e * self.veh.m_t ...
    + T_b./1000./ self.veh.r;


%% One Gear
if isempty(Cl)
    f_acc = f_acc ...
        + (T_1 - T_loss_m_1 - T_loss_gb_1)...
        ./1000 .* (self.veh.i_gr1_1)  ./ self.veh.r;
end

%% Two Gears
if ~isempty(Cl)
    
    if self.relaxed == 1
        f_acc = f_acc...
            + (T_1 - T_loss_m_1 - T_loss_gb_1) .*...
            (((2-Cl)./1000 .* self.veh.i_gr1_1 ./ self.veh.r)...
            +((Cl-1) ./1000 .* self.veh.i_gr1_2 ./ self.veh.r));
    else
        f_acc = f_acc...
            + (T_1 - T_loss_m_1 - T_loss_gb_1) .*...
            (((2-Cl) ./1000 .* self.veh.i_gr1_1 ./ self.veh.r)...
            +((Cl-1) ./1000 .* self.veh.i_gr1_2 ./ self.veh.r))...
            - 100.*(2-Cl).*(Cl-1);
    end
end

%% Two Motors
if ~isempty(T_2)
    
    % Mechanical  Motor Losses
    T_loss_m_2 = HelpFun.genPolyFun(self.veh.motor_2.fit_lossMech,n_2*2*pi,T_2);
    
    % GB losses
    if  self.splitGbMap == 1
        T_loss_gb_2_a = HelpFun.genPolyFun(self.veh.gb_2_losses.fit_sp,n_2*2*pi,T_2-T_loss_m_2);
        T_loss_gb_2_b = HelpFun.genPolyFun(self.veh.gb_2_losses.fit_sp,n_2*2*pi,-(T_2-T_loss_m_2));
        T_loss_gb_2 = Slack_GB2;
    else
        T_loss_gb_2_a = [];
        T_loss_gb_2_b = [];
        T_loss_gb_2 = HelpFun.genPolyFun(self.veh.gb_2_losses.fit_co,n_2*2*pi,T_2-T_loss_m_2);
    end
    
    
    f_acc = f_acc ...
        + (T_2  - T_loss_m_2 - T_loss_gb_2)...
        ./1000 .* (self.veh.i_gr2) ./ self.veh.r  ;
    
else
    T_loss_gb_2_a = [];
    T_loss_gb_2_b = [];
    T_loss_gb_2 = [];
    T_loss_m_2 = [];
end

end


