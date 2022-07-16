function [] = optiDrivingMission(self, dm, aux, stSOC)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Simulates the car-following scenario and handles the
% optimizer. After the initialization of the carEstimater, optimizer, and
% the empty solution variables, the skript loops though the time.
% ------------
% Input:    - self: OptiFrameMovingHorizon Object
%              - dm: struct, Driving Mission
%              - aux: float, Auxiliary power in [W]
%              - stSOC: float, Start SOC in [%]
% ------------

%% Initilize
if self.plotDynamic >= 1
figure
set(gcf,'color','w');
hold on
if self.plotDynamic == 2
    subplot(2,1,1)
    hold on
end
HelpFun.plotVmax(dm) 
end

self.dm=dm;
self.stSOC = stSOC;

% Carestimeter
self.carEst = ClassDefs.CarEstimater(self.dm,self.horizon,self.optionsCarEstimater);

self.optimizer = ClassDefs.Optimizer(self);
if self.twoGears==1
    self.optimizerRelaxed = ClassDefs.Optimizer(self,1);
end

% Solution Variable
self.resMh.T_1 = nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
self.resMh.T_b =  nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
self.resMh.P_b =  nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
self.resMh.v = nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq))+1,1);
self.resMh.v(1)=0;
self.resMh.s = nan(size(self.resMh.v ));
self.resMh.s(1)=0;
self.resMh.t = nan(size(self.resMh.v ));
self.resMh.t(1)=0;
self.resMh.SOC = stSOC*ones(size(self.resMh.v ));
self.resMh.a =  nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
if self.twoMotors == 1
    self.resMh.T_2 =  nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
    if self.splitGbMap  == 1
        self.resMh.Sl_loss_gb_2 =  nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
    end
end
if self.twoGears == 1
    self.resMh.Cl =  nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
end
if self.splitGbMap  == 1
    self.resMh.Sl_loss_gb =  nan(length(self.dm.t)/self.updateSeq*length(self.horizon(self.horizon<self.updateSeq)),1);
end

% Initalize Timer
self.time = zeros(length((0:self.updateSeq: dm.t(end))),1);
self.optiError = zeros(length((0:self.updateSeq: dm.t(end))),1);

if self.twoGears == 1
    self.timeRel = zeros(length((0:self.updateSeq: dm.t(end))),1);
end

old_percent = 0;

% Solution Length 
    lensolm =  length(self.horizon(self.horizon<=self.updateSeq));
    lensoln = lensolm-1;

%% first guess at point zero
self.res.v= zeros(size(self.horizon));
self.res.s = upd_s(self.horizon, self.res.v);
self.res.a =  zeros(length(self.horizon)-1,1);

indx = 1;
counter = 1;
%% LOOPING
for simt = 0:self.updateSeq: dm.t(end)

    
    current_percent = round(simt / dm.t(end)*100,-1);
    if current_percent~= old_percent
        old_percent = current_percent;
        cur_av_solv_time=sum(self.time)/numel(self.time(self.time~=0))*1000;
        disp(strcat('Vehicle is driving. Made: '," ", int2str(current_percent), '%'))
        disp(strcat('Current average solver time: '," ", int2str(cur_av_solv_time), ' ms'))
        if self.twoGears == 1
            cur_av_solv_time_rel=sum(self.timeRel)/numel(self.timeRel(self.timeRel~=0))*1000;
           disp(strcat('Current average solver time relaxed: '," ", int2str(cur_av_solv_time_rel), ' ms')) 
        end
    end
    
    
    % Get current status
    getCurrentState(self,indx)
    
    % estimate leading vehicle
    [self.est] = self.carEst.estimateVeh(simt);
    
    % Define bounds of x
    defineBoundsx(self)
    
    % Define variable params
    defineParamVariable(self,aux)
    
    % First guess
    definex0(self,counter)
    
    %Optimize
    [self.res,self.time(counter)] = self.optimizer.(strcat("optimiseProfile",self.indVar))(self.paramVariable, self.lbx, self.ubx,self.x0);
    if self.optimizer.solver.stats.success == 0
        disp('No optimal solution found')
        self.optiError(counter) = 1;
        disp('Initilize new x0')
        self.x0(2:end) = self.x0(1);
        disp('Redo Optimization')
        [self.res,self.time(counter)] = self.optimizer.(strcat("optimiseProfile",self.indVar))(self.paramVariable, self.lbx, self.ubx,self.x0);
        if self.optimizer.solver.stats.success == 1
            disp('New x0 succeeded')
        else
            disp('Still no optimal solution found')
        end
        
    end
    
    % Add part to overall solution
    indxm = indx+lensoln;
    indxn = indxm-1;
    
    self.resMh.s(indx:indxm)=self.res.s(1:lensolm);
    self.resMh.v(indx:indxm)=self.res.v(1:lensolm);
    self.resMh.SOC(indx:indxm)=self.res.SOC(1:lensolm);
    self.resMh.a(indx:indxn)=self.res.a(1:lensoln);
    self.resMh.T_1(indx:indxn)=self.res.T_1(1:lensoln);
    self.resMh.T_b(indx:indxn)=self.res.T_b(1:lensoln);
    self.resMh.P_b(indx:indxn)=self.res.P_b(1:lensoln);
    self.resMh.I_b(indx:indxn)=self.res.I_b(1:lensoln);
    
    if self.twoMotors == 1
        self.resMh.T_2(indx:indxn)=self.res.T_2(1:lensoln);
        if self.splitGbMap  == 1
            self.resMh.Sl_loss_gb_2(indx:indxn)=self.res.Sl_loss_gb_2(1:lensoln);
        end
    end
    if self.twoGears == 1
        Cl_plot =  reshape(repmat(self.res.Cl,uint8(1),uint8(self.n/self.nGear))',self.n,1);
        self.resMh.Cl(indx:indxn)=Cl_plot(1:lensoln);
    end
    if self.splitGbMap  == 1
        self.resMh.Sl_loss_gb(indx:indxn)=self.res.Sl_loss_gb(1:lensoln);
    end
    
    
    
    self.resMh.t(indx:indxm)=(self.horizon(1:lensolm))+  self.resMh.t(indx);
    
    indx      = indx+lensoln;
    
    % Plotting
    if self.plotDynamic >= 1
        dynamicPlot(self,simt,lensoln,lensolm)
    end
   
%     clf
%     hold on
%     plot(self.x0)
%     plot(full(self.optimizer.solution.x))

    counter=counter+1;
end

%% Postprocessing
self.res = self.resMh;
self.resMh = [];

calcEnergyQuasistatic(self)


end

function s0 = upd_s(t0,v0)
s0 = cumtrapz(t0,v0);
end

