function [] = defineParamVariable(self,aux)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Defines the bounds of the state and control vector x
% ------------
% Input:    - self: OptiFrameMovingHorizon Object
%              - aux: vehicle auxiliary power in [W] 
% ------------


if nargin ==1
    aux = 0;

end
self.paramVariable.delta_t_param = diff(self.horizon);
self.paramVariable.a_last_param = self.cs.a_last_param;
dt = diff(self.horizon(self.horizon<=self.updateSeq));
self.paramVariable.delta_t_last_param = dt(end);
self.paramVariable.p_aux_w = aux;
self.paramVariable.startSOC = self.cs.SOC;
self.paramVariable.s_lead_veh = self.est.s;
self.paramVariable.v_max = self.vMax;
self.paramVariable.T_1_last_param = self.cs.T_1_last_param;
self.paramVariable.T_b_last_param = self.cs.T_b_last_param;

if self.twoMotors == 1
self.paramVariable.T_2_last_param= self.cs.T_2_last_param;
end


target =  self.res.s(1);

difference = self.dm.v_max_s -target;

difference(difference>0) = -inf;
[~,idx] = max(difference);



s_relevant = self.dm.v_max_s(idx:end);
v_relevant = self.dm.v_max(idx:end);

indxend = min(length(s_relevant),self.numSpeedLim);

toadd =  max(self.numSpeedLim - indxend,0);

self.paramVariable.v_max_s_vec = [s_relevant(1:indxend);...
    s_relevant(indxend) * ones(toadd,1)];

self.paramVariable.v_max_vec = [v_relevant(1:indxend); ...
       v_relevant(indxend)* ones(toadd,1)];



end

