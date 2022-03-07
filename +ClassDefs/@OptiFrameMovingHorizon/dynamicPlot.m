function [] = dynamicPlot (self,simt,lensoln,lensolm)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Dynamic plot for vehicle car following
% ------------
% Input:    - self: OptiFrameMovingHorizon Object
%              - simt: current simulation time
%              - lensoln: length of driven solution for variables of length
%              n
%              - lensolm: length of driven solution for variables of length
%              m
% ------------




if self.plotDynamic == 2
    subplot(2,1,1)
    hold on
end



xlim([self.res.s(1)-50 self.res.s(end)+100])       % x limit
ylim([0 max(20,max(self.res.v)+2)])                %y limit

v_vc = interp1(self.dm.t,self.dm.lv_v,simt+1);
s_vc =  interp1(self.dm.t,self.dm.lv_s,simt+1);
scatter(self.res.s(1:lensolm),self.res.v(1:lensolm),'r','filled') % Driven distance
scatter(s_vc,v_vc,'b','filled')                             % Leading vehicle
plot(self.est.s,self.est.v,'k','LineWidth',1)           % Estimation
plot(self.res.s,self.res.v,'g')                             % optimal solution

if simt ==0
    hlegend=legend('Max Speed','Ego-vehicle','Leading vehicle','Leading Vehicle Prediction','Plan ego-vehicle','Location','northeast','AutoUpdate','off');
    xlabel('Distance in m')
    ylabel('Speed in m/s')
end



if self.plotDynamic == 2
    subplot(2,1,2)
    hold on
    
    yyaxis left
    plot(self.res.s(1:lensoln),self.res.T_1(1:lensoln),'b','LineWidth',1,'Marker', 'none');
    plot(self.res.s(1:lensoln),self.res.T_b(1:lensoln),'r','LineWidth',1,'Marker', 'none');
    ylim([min([-100;self.res.T_1]) max([100;self.res.T_1])])
    
  
    if self.twoGears == 1
          yyaxis right
        plot(self.res.s(1:lensoln),self.res.Cl(1:lensoln),'m','LineWidth',1,'Marker', 'none');
        ylim([0 3])
    elseif self.twoMotors == 1
        plot(self.res.s(1:lensoln),self.res.T_2(1:lensoln),'m','LineWidth',1,'Marker', 'none');
    end
    xlim([self.res.s(1)-50 self.res.s(end)+100])
    


if simt ==0
    if self.twoMotors == 1
        hlegend2=legend('Motor ','Brake','Motor 2','Location','northeast','AutoUpdate','off');
    elseif self.twoGears == 1
        hlegend2=legend('Motor ','Brake','Gear','Location','northeast','AutoUpdate','off');
        yyaxis right
        ylabel('Gear')
    else
        hlegend2=legend('Motor ','Brake','Location','northeast','AutoUpdate','off');
    end
    xlabel('Distance in m')
    yyaxis left
    ylabel('Torque in Nm')
    
end
end


end
