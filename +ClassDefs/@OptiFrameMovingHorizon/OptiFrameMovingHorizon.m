classdef OptiFrameMovingHorizon< ClassDefs.OptiFrame
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: OptiFrameMovingHorizon includes all relevent data to optimize the speed
% profile of a vehicle. OptiFrameMovingHorizon inherit from OptiFrame
% ------------
    
    properties
        resMh                                     % Result of Moving horizon cycle
        updateSeq                                    % Update frequency in [s]
        horizon                                      % Horizon as vector with all evaluation points
        carEst                                         % Object that estimate vehicle in front
        est                                                 % Estimation of leading vehicle
        cs                                                  % Current state of ego vehicle
        vMax                                       % Current v_max
        stSOC                                        % Start SOC of Simulation/MPC
        optionsCarEstimater               % Options for car estimater
        optiError                                  % Vector that contains possible errors of optimizations
        maxDistance;                           % Parameter if maximal distance should be constraint (only for MPC)
        plotDynamic;                           % Parameter, for dynamic plotting %1 for small plotts 2 for extended plots
    end
    
    properties (Constant)
        
    end
    
    methods
        % CONSTRUCTOR
        function self = OptiFrameMovingHorizon(indVar,veh,options)
            
            % Inheritate from Masterclass
            self = self@ClassDefs.OptiFrame('M',indVar,veh,options);
            
            %  Default Parameters MPC
            self.maxDistance = 0;
            self.optionsCarEstimater = 'V2V';
            self.resMh=[];
            self.updateSeq = 1;
            self.horizon = (0:0.2:10)';
            self.m = length(self.horizon);
            self.n = self.m-1;
            self.plotDynamic = 0;
            if self.twoGears == 1
                self.nGear = self.n;
            end
            
            % Evaluate MPC relevant options and overwrite default valus
            if isfield(options,'carEstimater')
                self.optionsCarEstimater = options.carEstimater;
            end
            if isfield(options,'solver')
                if isfield(options.solver,'maxDistance')
                    self.maxDistance = options.solver.maxDistance;
                end
            end
            if isfield(options,'plot')
                if isfield(options.plot,'dynamic')
                    self.plotDynamic = options.plot.dynamic;
                end
            end
            
            
        end
        
        function setHorizon(self, horizon)
            self.horizon = horizon;
            self.m = length(self.horizon);
            self.n = self.m-1;
            if self.twoGears == 1
                self.nGear = self.n;
            end
        end
    end
end

