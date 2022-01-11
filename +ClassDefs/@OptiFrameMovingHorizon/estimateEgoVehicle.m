function [v0,s0] = estimateEgoVehicle (self)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Estimates the movement of the ego vehicle for the next
% optimization run
% ------------
% Input:    - self: OptiFrameMovingHorizon Object
% ------------
% Output:    - v0: first guess velocity profile for next optimization run
%                 - s0: first guess distance profile for next optimization
%                 run
% ------------


    % Estima ego speed profile
    v0 = self.res.v(self.horizon>=self.updateSeq);
    a_add = self.res.a(end);
    numadd = length(self.horizon)-length(v0);
    v_add = v0(end) + max(cumsum(a_add.*diff(self.horizon(end-numadd:end))),0);
    v0 = [v0; v_add];
    s0 = self.cs.s  + upd_s(self.horizon, v0);
    if sum(self.est.s < s0) > 0
        s0 = min(s0,self.est.s');
    end
    v_max_fine = interp1(self.dm.v_max_s(:,1),self.dm.v_max_s(:,2),(s0(1):1:s0(end)));
    self.v_max = interp1(self.dm.v_max_s(:,1),self.dm.v_max_s(:,2),s0);
    [~,idxmin]=min(self.v_max);
    self.v_max(idxmin)=inf;
    [~,idxmin2]=min(self.v_max);
    self.v_max(idxmin)=min(v_max_fine);
    self.v_max(idxmin2)=min(v_max_fine);
   if sum(self.v_max < v0) > 0
        v0 = min(v0,self.v_max);
        s0 = self.cs.s  + upd_s(self.horizon,v0);
   end
    
end

function s0 = upd_s(t0,v0)
s0 = cumtrapz(t0,v0);
end