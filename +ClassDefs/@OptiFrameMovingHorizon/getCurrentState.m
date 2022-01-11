function []=getCurrentState(self,indx)
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
%              - indx: int, index of current run within whole solution
% ------------

% Time at optimisation start
cs.t = self.resMh.t(indx);
cs.s = self.resMh.s(indx);
cs.v = self.resMh.v(indx);
cs.SOC = self.resMh.SOC(indx);

% Acceleration of last time step
if indx == 1
    cs.a_last_param = 0;
else
    cs.a_last_param = self.resMh.a(indx-1);
end

% Torque of last time step
if indx == 1
    cs.T_1_last_param = 0;
    cs.T_b_last_param = 0;
    
    if self.twoMotors == 1
        cs.T_2_last_param = 0;
    end
    
else
    cs.T_1_last_param = self.resMh.T_1(indx-1);
    cs.T_b_last_param = self.resMh.T_b(indx-1);
    
    
    if self.twoMotors == 1
        cs.T_2_last_param = self.resMh.T_2(indx-1);
    end
end
self.cs=cs;

end

