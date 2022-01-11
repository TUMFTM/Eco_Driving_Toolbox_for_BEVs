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
        
        
    end
    
    properties (Constant)
        
    end
    
    methods
        function self = OptiFrameRoute(indVar,veh,options)
            if nargin == 1
                veh=[];
            end
            
            %% Predefined options for Route
            options.scalingwjwa = 0;
            
            %% Inheritate from Masterclass
            self = self@ClassDefs.OptiFrame('R',indVar,veh,options);
            
            if isfield(options.solver,'limitOverallRoute')
                self.limitOverallRoute = options.solver.limitOverallRoute;
            end
            
        end
        
        
        [] = defineBoundsx(self)
        []= defineParamVariable(self,aux,stSOC)
        [] = optiDrivingMission(self, dm, aux, stSOC, flag_nosim)
        definex0(self)
        
        
    end
    
    
end

