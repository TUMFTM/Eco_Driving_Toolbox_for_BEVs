 function [] = defineWeightParam(self, ...
     w_j,... %2
     w_a,... %3
     w_E,... %4
     w_r,... %5
     w_r_br,...  %6
     w_v,...    %7
     w_v_lim,...   %8
     w_vEnd,...  %9
     w_s,...%10
     w_cons)   %11
% Designed by: Alexander Koch and Tim Bürchner (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Defines the weighting paramets for the optimizer. Must be set
% before the solver is initilized. 
% ------------
% Input:    - w_j: Weight on jerk
%              - w_a: Weight on acceleration
%              - w_E: Weight on energy
%              - w_r: Weight on regularization of motor torque 
%              - w_r_br: Weight on regularization of braking torque 
%              - w_v: Weight on velocity (velocity maximization)
%              - w_v_lim: Weight on speed limit (minimization  diference between speedlimit and vehicle speed)
%              - w_vEnd: Weight on kinetic energy at end of horizon
%              - w_s: Weight on setting target distance to leading vehicle
%              - w_cons: Weight on vehicle consumption
% ------------

if nargin >= 11
    self.paramWeighting.w_cons = w_cons;
else
    self.paramWeighting.w_cons = 0;
end

if nargin >= 10
    self.paramWeighting.w_s = w_s;
else
    self.paramWeighting.w_s = 0.001;
end

if nargin >= 9
    self.paramWeighting.w_vEnd = w_vEnd;
else
    self.paramWeighting.w_vEnd = 1;
end

if nargin >= 8
    self.paramWeighting.w_v_lim = w_v_lim;
else
    self.paramWeighting.w_v_lim = 0;
end

if nargin >= 7
    self.paramWeighting.w_v = w_v;
else
    self.paramWeighting.w_v = 0;
end

if nargin >= 6
    self.paramWeighting.w_r_br = w_r_br;
else
    self.paramWeighting.w_r_br = 5e-7;
end

if nargin >= 5
    self.paramWeighting.w_r = w_r;
else
    self.paramWeighting.w_r = 5e-7;
end

if nargin >= 4
    self.paramWeighting.w_E = w_E;
else
    self.paramWeighting.w_E = 1;
end

if nargin >= 3
    self.paramWeighting.w_a = w_a;
else
    self.paramWeighting.w_a = 0.0;
end

if nargin >= 2
    self.paramWeighting.w_j = w_j;
else
    self.paramWeighting.w_j = 20;
end


end

