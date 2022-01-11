classdef Optimizer< handle
    % Designed by: Alexander Koch (FTM, Technical University of Munich)
    %-------------
    % Created on: 2019-2022
    % ------------
    % Version: Matlab2021a
    %-------------
    % Description: Optimizer is the optimizer of the eco-driving-toolbox. With
    % the help of casadi, the optimization problem is formulated and can be
    % solved on demand.
    % ------------
    
    properties
        type;               % Optimization type (R or M)
        indVar;             % Independent variable (s or t)
        m;                  % Discretization points
        n;                  % Discretization steps
        nGear;            % Discrete points where shifting is possible
        twoGears;           % Parameter if 2 gears are used
        twoMotors;          % Parameter if 2 motors are used
        x;              %State vector
        veh;                % Vehicle parameter
        paramConstant;     % Constant parameters
        paramWeighting;    % Weighting parameters
        scalingwjwa;          % Enable scaling of w_j and w_a
        time;                       % Calculation time
        splitGbMap;        % Parameter if splitted gearbox map should be used
        splitMotMap;        % Parameter if splitted motor map should be used
        limitOverallRoute; % limitation of jerk and acceleration over complete Route (only for Route optimization)
        constants;      % = struct('g_e', 9.81, 'roh_a', 1.18) % Gravity and density air
        solver;        % Solver
        relaxed;     % Parameter if the solve should solve the problem in a relaxed form
        solution;    % Solution
        numSpeedLim;     % Maximal Number of different speed limits within one optimization (smaller number is faster)
        minTorqueRecu;       % Parameter if minimal torque for recu should be considered
        maxDistance;       %  Parameter if maximal distance should be constraint (only for MPC)
        g;                       % Inequality contraints
       lbg;                        %   Lower bound of inequality contraints
        ubg;                           %   Upper bound of inequality contraints
       J;                               % Objective function
        
    end
    
    properties (Constant)
        
    end
    
    methods
        function self = Optimizer(OptiFrame,relaxed)
            if nargin == 1
                relaxed=[];
            end
            
            self.type = OptiFrame.type;               % Optimization type (R or MPC)
            self.indVar = OptiFrame.indVar;             % Independent variable (s or t)
            self.m = OptiFrame.m;                  % Discretization points
            self.n =   OptiFrame.n;                % Discretization steps
            self.nGear = OptiFrame.nGear;            % Discrete points where shifting is possible
            self.twoGears =OptiFrame.twoGears;           % parameter if 2 gears are used
            self.twoMotors = OptiFrame.twoMotors;         % parameter if 2 motors are used
            self.veh = OptiFrame.veh ;             % Vehicle parameter
            self.paramConstant = OptiFrame.paramConstant;     % constant parameters
            self.paramWeighting = OptiFrame.paramWeighting;    % weighting parameters
            self.scalingwjwa = OptiFrame.scalingwjwa;          % enable scaling of w_j and w_a
            self.constants=OptiFrame.constants;
            self.splitGbMap=OptiFrame.splitGbMap;
            self.splitMotMap=OptiFrame.splitMotMap;
            self.relaxed = relaxed;
            self.time = [];                       % Calculation time
            self.x = [];
            self.numSpeedLim =  OptiFrame.numSpeedLim;
            self.minTorqueRecu = OptiFrame.minTorqueRecu;
            
            %MPC and Route individual parameters
            if  isfield(OptiFrame,'limitOverallRoute')
                self.limitOverallRoute=OptiFrame.limitOverallRoute;
            else
                self.limitOverallRoute=0;
            end
            
            if  isfield(OptiFrame,'maxDistance')
                self.maxDistance = OptiFrame.maxDistance;
            else
                self.maxDistance = 0;
            end
            
            
            initSolver(self)
            
        end
        
        [] = initSolver(self)
        [f_acc,T_loss_m_1,T_loss_m_2,T_loss_gb_1,T_loss_gb_2,T_loss_gb_1_a,T_loss_gb_1_b,T_loss_gb_2_a,T_loss_gb_2_b]  = calcAccForce(self, v, T_1, T_b, T_2, n_1,n_2, Cl,Slack_GB,Slack_GB2)
        [res, time]= optimiseProfilet(self, paramVariable, lbx, ubx,x0)
        [res] = extractSolution(self)
    end
    
    
    
end
