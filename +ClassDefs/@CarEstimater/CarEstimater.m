classdef CarEstimater< handle
   %CarEstimater: Class to predict the longitudinal movement of the leading
   %vehicle.
   % Author:   Alexander Koch, Ph.D. Candidate, TU Munich, Institute for Automotive Technologie
   % Date:     11.01.22

    
    properties
        dm                                       % Driving Mission
        horizon                                 % Horizon
        est                                        % Estimation of vehicle
        predType                            % Predivtion type ('V2V' or 'a')
        vLast                                      % Velocity of last time step
        tLast                                      % Time of last time step
    end
    
    properties (Constant)
    end
    
    methods
        function self = CarEstimater(dm,horizon,predType)

            self.dm=dm;
            self.horizon=horizon;
            self.predType = predType;
            self.vLast = 0;
            self.tLast = -1;
            
        end
        
        
        [est] = estimateVeh(self,time)

        
    end
    
    
    
end
