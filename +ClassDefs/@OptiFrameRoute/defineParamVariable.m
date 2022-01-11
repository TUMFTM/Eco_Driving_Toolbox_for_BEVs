function [] = defineParamVariable(self,aux,stSOC)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Defines the bounds of the state and control vector x
% ------------
% Input:    - self: OptiFrameRoute Object
%              - aux: vehicle auxiliary power in [W] 
%              - stSOC: float, Start SOC in [%]
% ------------




if nargin ==1
    aux = 0;
    stSOC=100;
end
self.paramVariable.delta_t_param = diff(self.dm.t);
self.paramVariable.a_last_param = self.dm.a_last_param;
self.paramVariable.delta_t_last_param =  self.paramVariable.delta_t_param(1);
self.paramVariable.a_next_param = self.dm.a_next_param;
self.paramVariable.delta_t_next_param =  self.paramVariable.delta_t_param(end);
if self.limitOverallRoute == 1
    self.paramVariable.a_max_sq_sum = self.dm.a_max_sq_sum;
    self.paramVariable.j_max_sq_sum = self.dm.j_max_sq_sum;
end

self.paramVariable.p_aux_w = aux;
self.paramVariable.startSOC = stSOC;

self.paramVariable.v_max_s_vec = self.dm.v_max_s;
    
toadd =  self.numSpeedLim - length(self.dm.v_max_s);

self.paramVariable.v_max_s_vec = [self.paramVariable.v_max_s_vec;...
    self.paramVariable.v_max_s_vec(end) * ones(toadd,1)];

self.paramVariable.v_max_vec = [self.dm.v_max; ...
       self.dm.v_max(end) * ones(toadd,1)];
    
end

