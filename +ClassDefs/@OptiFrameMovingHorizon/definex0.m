function []= definex0(self, counter)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Preparations for first guess and call of firstguess function
% ------------
% Input:    - self: OptiFrameMovingHorizon Object
%              - counter: counter for relaxed optimization frame
% ------------

v0 = self.res.v(self.horizon>=self.updateSeq);
a_add = self.res.a(end);
numadd = length(self.horizon)-length(v0);
v_add = v0(end) + max(cumsum(a_add.*diff(self.horizon(end-numadd:end))),0);
v0 = [v0; v_add];

s0 = self.cs.s+ cumsum([0; v0(1:end-1).*diff(self.horizon)]);
if length(self.dm.v_max)>1
    v_max_0 = interp1(self.dm.v_max_s,self.dm.v_max,s0,'previous');
else
    v_max_0 = self.dm.v_max;
end

if sum(v0>v_max_0)>=1
    v0=min(v0,v_max_0);
end


firstguess(self,v0,counter);

end

