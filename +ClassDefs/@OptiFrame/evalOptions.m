function [] = evalOptions(self,options)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Evaluation of options
% ------------
% Input:    - self: OptiFrame
%              - options: struct with options (please see README)
% ------------

if isfield(options,'twoGears')
    self.twoGears = options.twoGears;
end

if isfield(options,'twoMotors')
    self.twoMotors = options.twoMotors;
end

if isfield(options,'solver')
    
    if isfield(options.solver,'splitGbMap')
        self.splitGbMap = options.solver.splitGbMap;
    end
    
    if isfield(options.solver,'splitMotMap')
        self.splitMotMap = options.solver.splitMotMap;
    end
    
    if isfield(options.solver,'scalingwjwa')
        self.scalingwjwa = options.solver.scalingwjwa;
    end
    
    if isfield(options.solver,'minTorqueRecu')
        self.minTorqueRecu = options.solver.minTorqueRecu;
    end
    
end

if isfield(options,'plot')
    if isfield(options.plot,'results')
        self.plotResults = options.plot.results;
    end
end

end

