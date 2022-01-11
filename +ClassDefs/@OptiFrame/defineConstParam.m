function [] = defineConstParam(self, a_max_t, a_max_b, j_max, t_ds, t_dmin, t_dmax, s_max_v0, s_min_v0)
% Designed by: Alexander Koch and Tim Bürchner (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Defines the constat paramets for the optimizer. Must be set
% before the solver is initilized. Please see corresponding paper for the
% exact target distance to the leading vehicle based on t_ds, t_dmin,
% t_dmax, s_max_v0, s_min_v0
% ------------
% Input:    - a_max_t: Maximum acceleration traction in [m/s^2]
%              - a_max_b: Maximum acceleration braking in [m/s^2]
%              - j_max: Maximum jerk [m/s^3]
%              - t_ds: Target time gap to leading vehicle in  [s]
%              - t_dmin: Minimum time gap to leading vehicle in [s]
%              - t_dmax: Maximum time gap to leading vehicle in [s]
%              - s_max_v0: Maximum distance to leading vehicle at
%              standstill in [m]
%              - s_min_v0: Minimum distance to leading vehicle at
%              standstill in [m]
% ------------

if  strcmp(self.type,'R')
    if nargin == 1
        self.paramConstant.a_max_t = 2;
        self.paramConstant.a_max_b = -3.5;
        self.paramConstant.j_max = 2;
    elseif nargin == 4 || 9
        self.paramConstant.a_max_t = a_max_t;
        self.paramConstant.a_max_b = a_max_b;
        self.paramConstant.j_max = j_max;
    end
end
if  strcmp(self.type,'M')
    if nargin == 1
        self.paramConstant.a_max_t = 5;
        self.paramConstant.a_max_b = -5.5;
        self.paramConstant.j_max = 3;
        self.paramConstant.t_ds = 1.8;
        self.paramConstant.t_dmin = 1.0;
        self.paramConstant.t_dmax = 5;
        self.paramConstant.s_max_v0 = 3;
        self.paramConstant.s_min_v0 = 1.5;
    elseif nargin ==9
        self.paramConstant.a_max_t = a_max_t;
        self.paramConstant.a_max_b = a_max_b;
        self.paramConstant.j_max = j_max;
        self.paramConstant.t_ds = t_ds;
        self.paramConstant.t_dmin = t_dmin;
        self.paramConstant.t_dmax = t_dmax;
        self.paramConstant.s_max_v0 = s_max_v0;
        self.paramConstant.s_min_v0 = s_min_v0;
    end
    
    if nargin == (~1 || ~4 || ~9)
        error('1,4 or 9 inputs including self are needed')
    end
    
end

