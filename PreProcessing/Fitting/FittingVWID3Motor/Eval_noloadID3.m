clear

load('C:\Users\ga73boc\Downloads\track_1500000309481.mat')

    VarNames = cellstr(columns);
    RawData = array2table(data , 'VariableNames', VarNames);
    
    figure
    hold on
    plot(RawData.speed)
    
    xs{1,1} = 1210;
    xe{1,1} = 2041;
    
    xs{1,2} = 9602;
    xe{1,2}  = 9715;
          
    
    
    %% Cut relevant data
    Vars{1}.Revolutions.RPM_engine = RawData.engine_rpm;
    %Battery
    Vars{1}.Voltages.U_Battery_HV = RawData.hv_battery_voltage;
    Vars{1}.Currents.I_Battery_HV = RawData.hv_battery_current;
    Vars{1}.Power.P_Battery = -((Vars{1}.Voltages.U_Battery_HV.*Vars{1}.Currents.I_Battery_HV));
    %P_aux
    %Vars.Power.P_aux = mean(RawData.hv_aux_power)/1000; %in kW
    Vars{1}.Power.P_aux = RawData.dcdc_current_lv.*RawData.dcdc_voltage_lv;

    %P_Inverter
    Vars{1}.Power.P_Inverter = RawData.inverter_dc_link_voltage.*RawData.inverter_dc_link_current;%Vars{1}.Power.P_Battery-Vars{1}.Power.P_aux;
    
      %Motor Speed
    Vars{1}.Power.speed_motor = RawData.engine_rpm;
    
    
    %
     Vars{1}.Power.test = RawData.inverter_dc_link_voltage.*RawData.inverter_dc_link_current;
     
     %figure
   % plot( Vars{1}.Power.test(xs{1,1}:xe{1,1}))
    
    %%
    
  %%  Second run
       load('C:\Users\ga73boc\Downloads\track_1500000309480.mat')
    
           VarNames = cellstr(columns);
    RawData = array2table(data , 'VariableNames', VarNames);
    
    figure
    hold on
    plot(RawData.speed)
    
    xs{2,1} = 14157;
    xe{2,1} = 14745;
    
    xs{2,2} = 14867;
    xe{2,2}  = 15180;
    
    
    
        %% Cut relevant data
    Vars{2}.Revolutions.RPM_engine = RawData.engine_rpm;
    %Battery
    Vars{2}.Voltages.U_Battery_HV = RawData.hv_battery_voltage;
    Vars{2}.Currents.I_Battery_HV = RawData.hv_battery_current;
    Vars{2}.Power.P_Battery = -((Vars{2}.Voltages.U_Battery_HV.*Vars{2}.Currents.I_Battery_HV));
    %P_aux
    %Vars.Power.P_aux = mean(RawData.hv_aux_power)/2000; %in kW
    Vars{2}.Power.P_aux = RawData.dcdc_current_lv.*RawData.dcdc_voltage_lv;

    %P_Inverter
    Vars{2}.Power.P_Inverter =RawData.inverter_dc_link_voltage.*RawData.inverter_dc_link_current;% Vars{2}.Power.P_Battery-Vars{2}.Power.P_aux;
    
      %Motor Speed
    Vars{2}.Power.speed_motor = RawData.engine_rpm;
    
    
    
    
    %% eval
    
    r = 2
    i = 1
    figure
    hold on
    plot(Vars{r}.Power.P_Inverter(xs{r,i}:xe{r,i}))
    plot(Vars{r}.Power.P_aux(xs{r,i}:xe{r,i}))
     plot(Vars{r}.Power.P_Battery(xs{r,i}:xe{r,i}))
     
         
     %scatter relevant data
          figure
          hold on
          VecFitx = [];
          VecFitz = [];
          for r = 1:2
         for i = 1:2
     VecFitx = [VecFitx; Vars{r}.Power.speed_motor(xs{r,i}:xe{r,i})];
     VecFitz = [VecFitz; Vars{r}.Power.P_Inverter(xs{r,i}:xe{r,i})];
     
     scatter(Vars{r}.Power.speed_motor(xs{r,i}:xe{r,i}),Vars{r}.Power.P_Inverter(xs{r,i}:xe{r,i}))
         end
     end
     
     %% Fittingparameters
    PARA.polyX = 3;
    PARA.polyY = 0;
    PARA.Method = 'min SE';
    PARA.FlipIneq = 1;
    
    mean_low = mean(Vars{1}.Power.P_Inverter(xs{1,2}:xe{1,2}));
    
    x_ineq = [0 500 1000 1500];
    z_ineq =0*mean_low*ones(size(x_ineq));
    
    
    
    
    %% Do fitting motor losses
[fitFun,~] = ...
    HelpFun.fittingEDT(VecFitx,zeros(size(VecFitx)),VecFitz,PARA,[],[],[],x_ineq,x_ineq,z_ineq)
    
% Since 2268 rpm is smallest measurement point,  

x=(1:1:14000);
plot(x,fitFun(x,zeros(size(x))))

x_rpm = x;
P_loss = fitFun(x_rpm,zeros(size(x_rpm)));
[minval, idx] = min(P_loss)
P_loss(1:idx)=minval;

ZeroTm.P = P_loss;
ZeroTm.n = x_rpm;









    