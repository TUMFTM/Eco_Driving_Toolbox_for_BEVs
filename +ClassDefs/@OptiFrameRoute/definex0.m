function definex0(self)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Preparations for first guess and call of firstguess function
% ------------
% Input:    - self: OptiFrameRoute Object
% ------------


% Inital speed profile guess
if isfield(self.dm,'v0')
        v_guess=self.dm.v0;
else
    v_guess=[0.5*(self.dm.ubv+self.dm.lbv)];
    
    if max(abs(diff(v_guess)./diff(self.dm.t)))>10
   v_guess2= interp1([0 1], [self.dm.ubv(1) self.dm.lbv(end)], linspace(0,1,length(self.dm.ubv)))';
    v_guess=(v_guess+v_guess2)/2;
end

if strcmp(self.indVar,'t')
    firstguess(self, v_guess);
else
    %
end
disp('Start vector defined.')

end

