function [] = plotFun(self)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: plots results of optimization
% ------------
% Input:    - self: OptiFrameObject
% ------------


if strcmp(self.type,'R')
t = self.dm.t;
ts =t(1:end-1);
end
if strcmp(self.type,'M')
t = self.res.t;
ts =t(1:end-1);
end


%% Plot comparison between T and SOC of res and resQs
figure

subplot(2,2,1)
hold on
grid on
plot(t,self.res.v,'LineWidth',2,'LineStyle','-')
legend('Vehicle Speed')

xlabel('Time in s','FontSize',12)
ylabel('Vehicle speen in m/s','FontSize',12)


subplot(2,2,2)
hold on
grid on
plot(ts,self.res.T_1,'LineWidth',2,'LineStyle','-')
plot(ts,self.resQs.T_1,'LineWidth',2,'LineStyle',':')
plot(ts,self.res.T_b/10,'LineWidth',2,'LineStyle','-')
legend('Optimizer Motor 1','Simulation Motor 1', 'Brake')

if self.twoMotors
    plot(ts,self.res.T_2,'LineWidth',2,'LineStyle','-')
    plot(ts,self.resQs.T_2,'LineWidth',2,'LineStyle',':')
    legend('Optimizer Motor 1','Simulation Motor 1','Optimizer Motor 2','Simulation Motor 2')
end
if self.twoGears
     yyaxis right
     if size(self.res.Cl) == size(self.res.T_1)
         Cl_plot = self.res.Cl;
     else
     Cl_plot =  reshape(repmat(self.res.Cl,uint8(1),uint8(self.n/self.nGear))',self.n,1);
     end
     stairs(ts,Cl_plot,'LineWidth',2)
     legend('Optimizer Motor 1','Simulation Motor 1', 'Brake', 'Gear')
     xlabel('Gear','FontSize',12)
     ylim([0 3])
     yyaxis left
end

xlabel('Time in s','FontSize',12)
ylabel('Motor air gap torque in Nm','FontSize',12)



subplot(2,2,3)
hold on
grid on
plot(t,self.res.SOC,'LineWidth',2,'LineStyle','-')
plot(t,self.resQs.SOC,'LineWidth',2,'LineStyle',':')
legend('Optimizer SOC','Simulation SOC')

xlabel('Time in s','FontSize',12)
ylabel('SOC in %','FontSize',12)

subplot(2,2,4)
hold on
grid on
plot(t,[0; cumsum(self.res.P_b.*diff(t))]./3600,'LineWidth',2,'LineStyle','-')
plot(t,self.resQs.Energy.E_cum./3600000,'LineWidth',2,'LineStyle',':')
legend('Optimizer Energy','Simulation Energy')

xlabel('Time in s','FontSize',12)
ylabel('Energy in kWh','FontSize',12)



end

