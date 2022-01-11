function [] = optiDrivingMission(self, dm, aux, stSOC)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Starts optimization of the driving mission
% ------------
% Input:    - self: OptiFrameRoute Object
%              - dm: struct, Driving Mission
%              - aux: float, Auxiliary power in [W]
%              - stSOC: float, Start SOC in [%]
% ------------


self.dm=dm;

%Udate m and n
self.m = length(dm.(self.indVar));
self.n = self.m-1;
if self.twoGears == 1
    self.nGear = self.n/4;
end
    

   	self.optimizer = ClassDefs.Optimizer(self);
    if self.twoGears==1
        self.optimizerRelaxed = ClassDefs.Optimizer(self,1);
    end   
    
    %% Manage variable input
    defineBoundsx(self)
    defineParamVariable(self,aux,stSOC)
    definex0(self)
    [self.res,self.time] = self.optimizer.(strcat("optimiseProfile",self.indVar))(self.paramVariable, self.lbx, self.ubx,self.x0);
    calcEnergyQuasistatic(self)

end

