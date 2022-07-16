classdef OptiFrame< handle
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: OptiFrame includes all relevent data to optimize the speed
% profile of a vehicle. OptiFrame inherit to OptiFrameRoute and
% OptiFrameMovingHorizon.
% ------------

    
    properties
        type;                           % Optimization type (R or M)
        indVar;                         % Independent variable (only t is supported)
        m;                                  % Discretization points
        n;                                      % Discretization steps
        nGear;                             % Discrete points where shifting is possible
        twoGears;                           % Parameter if 2 gears are used
        twoMotors;                          % Parameter if 2 motors are used
        optimizer;                          % Optimizer definition
        optimizerRelaxed;              % Optimizer definition for relaxed problem formulation
        lbx;                                    % Lower bound for the state vector
        ubx;                                    % Upper bound for the state vector
        veh;                                    % Vehicle parameter
        paramConstant;                 % Constant parameters
        paramWeighting;                % Weighting parameters
        paramVariable;                     % Variable parameters
        scalingwjwa;                      % Enable scaling of w_j and w_a
        res;                                        % Results of optimization
        resQs;                                     % Results of quasi-static simulation
        time;                                       % Calculation time
        timeRel;                           % Calculation time of relaxed problem
        splitGbMap;                    % Parameter if splitted gearbox map should be used
        splitMotMap;                   % Parameter if splitted motor map should be used
        constants;                                 % Gravity and density air
        dm;                                             % Driving Mission: Boundary conditions of driving scenario
        numSpeedLim                       % Maximal Number of different speed limits within one optimization (smaller number is faster)
        boundsx;                                   % Bounds of state vector x
        x0;                                             % First guess
        plotResults;                               % Parameter if results should be plotted
        minTorqueRecu;                          % Parameter if minimal torque for recu should be considered

    end
    
    properties (Constant)
        
    end
    
    methods
        % CONSTRUCTOR
        function self = OptiFrame(type, indVar, veh, options)
            
            
            self.type = type;               % Optimization type (R or MH)
            self.indVar= indVar;             % Independent variable (s or t --> only t is supported)
            
            % Default Parameters
            self.twoGears = 0;           
            self.twoMotors = 0; 
            self.splitGbMap = 1;
            self.splitMotMap = 1;
             self.minTorqueRecu = 0;
            self.constants = struct('g_e', 9.81, 'roh_a', 1.18);
            self.optimizer = [];             
            self.lbx = [];                
            self.ubx = [];              
            self.veh = [];                
            self.scalingwjwa = [];          
            self.res = [];                        
            self.resQs = [];                 
            self.time = [];                       
            self.dm = []; 
            self.boundsx=[];
            self.plotResults = 1;
            
            % Default Parameters based on type
            if strcmp(self.type,'R')
                self.numSpeedLim = 14;
                self.scalingwjwa = 0;
            elseif strcmp(self.type,'M')
                self.numSpeedLim = 4;
                self.scalingwjwa = 0;
            end
            
            % Overwrite Default Parameters with custom options
            evalOptions(self,options)
            
            % Load parameter
            defineConstParam(self)
            defineWeightParam(self)
            
            % Load vehicle
            loadVehicleParam(self, veh)           
        end
        
        
    end
    
    
    
end
