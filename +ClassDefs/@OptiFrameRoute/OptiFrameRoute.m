classdef OptiFrameRoute< ClassDefs.OptiFrame
    % Designed by: Alexander Koch (FTM, Technical University of Munich)
    %-------------
    % Created on: 2019-2022
    % ------------
    % Version: Matlab2021a
    %-------------
    % Description: OptiFrameRoute includes all relevent data to optimize the speed
    % profile of a vehicle. OptiFrameMovingHorizon inherit from OptiFrame
    % ------------
    
    properties
        
        limitOverallRoute;                % limitation of jerk and acceleration over complete Route (only for Route optimization)
        limitIWR;                                       % Parameter if Inertial work rating should be limited (For Cycle Beating)
        limitRMSSEv;                                % Parameter if RMSSE for v-ref should be limited (For Cycle Beating)
        
        
    end
    
    properties (Constant)
        
    end
    
    methods
        function self = OptiFrameRoute(indVar,veh,options)
            if nargin == 1
                veh=[];
            end
            
            
            %% Inheritate from Masterclass
            self = self@ClassDefs.OptiFrame('R',indVar,veh,options);
            
            if isfield(options.solver,'limitOverallRoute')
                self.limitOverallRoute = options.solver.limitOverallRoute;
            else
                self.limitOverallRoute = 0;
            end
            
            if isfield(options.solver,'limitIWR')
                self.limitIWR = options.solver.limitIWR;
            else
                self.limitIWR = 0;
            end
            
            
            if isfield(options.solver,'limitRMSSEv')
                self.limitRMSSEv = options.solver.limitRMSSEv;
            else
                self.limitRMSSEv = 0;
            end
            
            
            
            
            
            
        end
        
        
        [] = defineBoundsx(self)
        []= defineParamVariable(self,aux,stSOC)
        [] = optiDrivingMission(self, dm, aux, stSOC, flag_nosim)
        definex0(self)
        
        
    end
    
    
end

