function [res, time] = optimiseProfilet(self, paramVariable, lbx, ubx, x0)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: manages paramVariable and starts optimization process
% ------------
% Input:    - self: Optimizer Object
%              - paramVariable: struct, varable Parameters
%              - lbx: vector, lower bound of state and control vector
%              - ubx: vector, upper bound of state and control vector
%              - x0: vector, first guess
% ------------
% Output: - res: struct. solution of optization
%              - time: float, optimization time of solver
% ------------



%% Generate Param Vector
if self.type =='R'%isfield(paramVariable,'a_next_param')
    paramVec = vertcat(paramVariable.delta_t_param,...
        paramVariable.a_last_param,...
        paramVariable.delta_t_last_param,...
        paramVariable.p_aux_w,...
        paramVariable.startSOC,...
        paramVariable.v_max_s_vec,...
        paramVariable.v_max_vec,...
        paramVariable.a_next_param,...
        paramVariable.delta_t_next_param);
    
    if self.limitOverallRoute == 1
        paramVec = vertcat(paramVec,...
            paramVariable.a_max_sq_sum,...
            paramVariable.j_max_sq_sum);
    end
    
    
else %% MovingHorizon/MPC
    paramVec = vertcat(paramVariable.delta_t_param,...
        paramVariable.a_last_param,...
        paramVariable.delta_t_last_param,...
        paramVariable.p_aux_w,...
        paramVariable.startSOC,...
        paramVariable.v_max_s_vec,...
        paramVariable.v_max_vec,...
        paramVariable.s_lead_veh,...
        paramVariable.T_1_last_param,...
        paramVariable.T_b_last_param);
    
    if self.twoMotors == 1
        paramVec= vertcat(paramVec,...
            paramVariable.T_2_last_param);
    end
    
end

% Solve Problem
tic;
self.solution = self.solver('x0', x0', 'lbg',self.lbg, 'ubg', self.ubg, 'lbx', lbx, 'ubx', ubx, 'p', paramVec);
self.time = toc;

res = extractSolution (self);

time = self.time;


end

