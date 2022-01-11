function [] = calcEnergy_quasistatic(self)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Computes the energy consupmtion with a quasi-static
% simulation, using the results of the optimization and the tabulated loss
% maps. Battery losses are calculated with forward-simulation
% ------------
% Input:    - self: OptiFrame Object
% ------------



%% Interface

% Interface Route
if strcmp(self.type,'R')
    [v, a, dt, ~, aux, stSOC,n_1,n_2, T_1, T_2, T_b, Cl,Slack_GB,Slack_GB2] = interfaceR(self);
end

% Interface MPC
if strcmp(self.type,'M')
    [v, a, dt, ~, aux, stSOC,n_1,n_2, T_1, T_2, T_b, Cl,Slack_GB,Slack_GB2] = interfaceM(self);
end


%% Calc Air and Roll Resistance
F_roll = (self.veh.f_r_c0 + self.veh.f_r_c1 * v+self.veh.f_r_c2 *v.^2) * self.constants.g_e * self.veh.m_t *1000;
F_air =  0.5 * self.veh.c_a * self.veh.a_a * self.constants.roh_a * v.^2;
F_slope = 0;
F_acc = [a * self.veh.m_t * self.veh.lambda * 1000; 0];
F_wheel = F_roll + F_air + F_slope + F_acc;
F_wheel(v==0)=0;

%% Distribution Front/Rear based on share of traction torque at wheel from optimization
if nargin <= 2
% Mechanical Losses - Optimization
[~,T_loss_m_1,T_loss_m_2,T_loss_gb_1,T_loss_gb_2,T_loss_gb_1_a,T_loss_gb_1_b,T_loss_gb_2_a,T_loss_gb_2_b] =self.optimizer.calcAccForce(v, T_1, T_b, T_2, n_1,n_2, Cl,Slack_GB,Slack_GB2);

%Handle splitted gearbox
if self.splitGbMap ==1
    T_loss_gb_1 = max(T_loss_gb_1_a,T_loss_gb_1_b);
    if self.twoMotors == 1
        T_loss_gb_2 = max(T_loss_gb_2_a,T_loss_gb_2_b);
    else
        T_loss_gb_2 = 0;
    end
end



if self.twoGears == 0
    T_wheel_1 = (T_1 - T_loss_m_1- T_loss_gb_1) * self.veh.i_gr1_1;
else
    T_wheel_1 = (T_1 -T_loss_m_1- T_loss_gb_1) .*((2-Cl) .* self.veh.i_gr1_1 + (Cl-1) .* self.veh.i_gr1_2 );
end
if self.twoMotors == 1
    T_wheel_2 = (T_2 -T_loss_m_2- T_loss_gb_2) * self.veh.i_gr2;
else
    T_wheel_2 = 0;
end
   
% Share of motor
share_mot_1 = T_wheel_1./(T_wheel_1+T_wheel_2+T_b);
share_mot_2 = T_wheel_2./(T_wheel_1+T_wheel_2+T_b);

else
    share_mot_1 = 1;
    share_mot_2 = 0;
end
    
%% Motor 1 backwards calculation
F_wheel_1 = F_wheel(1:end-1) .*share_mot_1;
F_wheel_1(v(1:end-1)<0.01)=0;

% Quasistatic Simulation
if self.twoGears == 1
    gearratio1 = self.veh.i_gr1_1 .* (2-Cl) + self.veh.i_gr1_2 .* (Cl-1) ;
else
    gearratio1 = self.veh.i_gr1_1 ;
end

[P_mot1,P_loss_mot_el1, P_loss_mot_mech1, P_loss_gb1,T_1,n_1] = quasistaticAxleSim(self.veh, self.veh.motor_1, self.veh.gb_1_losses,gearratio1, F_wheel_1,v);

%% Motor 2 backwards calculation
if self.twoMotors == 1
    
    F_wheel_2 = F_wheel(1:end-1) .*share_mot_2;
    F_wheel_2(v(1:end-1)<0.01)=0;
    
    % Quasistatic Simulation
    [P_mot2,P_loss_mot_el2, P_loss_mot_mech2, P_loss_gb2,T_2,n_2] = quasistaticAxleSim(self.veh, self.veh.motor_2, self.veh.gb_2_losses,self.veh.i_gr2, F_wheel_2,v);
    self.resQs.T_2=T_2;
    
else
    P_mot2=0;
end

%% Battery forward simulation

P_bat_out = P_mot1+P_mot2 + aux*ones(size(P_mot1)); % required battery output power

[P_bat,P_loss_bat,SOC] = calcBatteryLosses(self,P_bat_out,stSOC,dt);

E_bat = [0; dt .* P_bat];

E_bat_out = [0; dt .* P_bat_out];

%% Save results

E_cum = cumsum(E_bat);

E_cum_bat_out = cumsum(E_bat_out);

s=sum((v(1:end-1)+v(2:end))*0.5.*dt/1000); % Distance in km


% Write Loss Struct
Losses.Energy.E_loss_bat = sum(P_loss_bat.*dt)/3600000;
Losses.Energy.E_loss_mot1 = sum((P_loss_mot_el1 + P_loss_mot_mech1).*dt)/3600000;
Losses.Energy.E_loss_mot1_el = sum((P_loss_mot_el1).*dt)/3600000;
Losses.Energy.E_loss_mot1_mech = sum((P_loss_mot_mech1).*dt)/3600000;
Losses.Energy.E_loss_gb1 = sum((P_loss_gb1).*dt)/3600000;
Losses.Energy.E_loss_WtD = sum(((F_roll(1:end-1)+F_air(1:end-1)).*v(1:end-1)).*dt)/3600000;
if self.twoMotors == 1
    Losses.Energy.E_loss_mot2 = sum((P_loss_mot_el2 + P_loss_mot_mech2).*dt)/3600000;
    Losses.Energy.E_loss_mot2_el = sum((P_loss_mot_el2 ).*dt)/3600000;
    Losses.Energy.E_loss_mot2_mech = sum((P_loss_mot_mech2).*dt)/3600000;
    Losses.Energy.E_loss_gb2 = sum((P_loss_gb2).*dt)/3600000;
end
Losses.Energy.E_aux = sum((aux.*ones(size(P_loss_bat))).*dt)/3600000;
Losses.Energy.Brake = abs(sum((T_b*self.veh.r.*v(1:end-1)).*dt))/3600000;

if self.plotResults == 1
    plotLosses(Losses,self.twoMotors)
end

% Check
E_loss_tot = Losses.Energy.E_loss_bat +Losses.Energy.E_loss_mot1 +Losses.Energy.E_loss_gb1+Losses.Energy.E_loss_WtD +Losses.Energy.E_aux +Losses.Energy.Brake;
if self.twoMotors == 1
    E_loss_tot = E_loss_tot + Losses.Energy.E_loss_mot2+ Losses.Energy.E_loss_gb2 ;
end

if abs((E_loss_tot(end)-E_cum(end)/3600000)/E_cum(end)/3600000) >0.01
    warning(strcat('abs((E_loss_tot(end)-E_cum(end))/E_cum(end))=',num2str(abs((E_loss_tot(end)-E_cum(end))/E_cum(end))),'<0.01'))
end

% Write Energy Struct
Energy.E_kWh = E_cum(end)/1000/3600;
Energy.E_kWhp100km = E_cum(end)/1000/3600 *100/s;
Energy.E_cum = E_cum;
Energy.E_cum_bat_out = E_cum_bat_out;
Energy.E_bat_out = E_bat_out;

% Write Power Struct
Power.Bat_out = P_bat_out;
Power.Bat = P_bat;
Power.Inv1 = P_mot1;
Power.Inv2 = P_mot2;

% Write in Obj
self.resQs.T_1=T_1;
self.resQs.SOC = SOC;
self.resQs.Energy=Energy;
self.resQs.Power = Power;
self.resQs.Losses = Losses;
end

function [v, a, dt, t, aux, stSOC,n_1,n_2, T_1, T_2, T_b, Cl,Slack_GB,Slack_GB2] = interfaceR(self)
v=self.res.v(~isnan(self.res.v)); % Velocity
a=self.res.a; % get acceleration
dt =self.paramVariable.delta_t_param; %get dt
t = [0; cumsum(dt)];
aux = self.paramVariable.p_aux_w;
stSOC = self.paramVariable.startSOC;
T_1 = self.res.T_1;
T_b = self.res.T_b;
%Motorrevs Motor 1
n_1 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_1; % Motor 1 speed  in [1/s]

% Slack GB
if self.splitGbMap ==1
    Slack_GB=self.res.Sl_loss_gb;
    if self.twoMotors == 1
        Slack_GB2 = self.res.Sl_loss_gb_2;
    else
        Slack_GB2=[];
    end
else
    Slack_GB=[];
    Slack_GB2=[];
end

if self.twoMotors == 0
    n_2=[];
    T_2=[];
end
if self.twoMotors == 1
    %Motorrevs Motor 2
    n_2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr2; % Motor 2 speed  in [1/s]
    T_2 = self.res.T_2;
end
    
    if self.twoGears==0
        Cl = [];
    elseif self.twoGears == 1
        Cl = self.res.Cl;
        Cl =reshape(repmat(Cl,uint8(1),uint8(self.n/self.nGear))',self.n,1);
        n_1_2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_2;
        n_1 = (2-Cl) .* n_1+ (Cl-1) .*  n_1_2;  % Actual Motor speed as function of clutch position
    end


end

function   [v, a, dt, t, aux, stSOC,n_1,n_2, T_1, T_2, T_b, Cl,Slack_GB,Slack_GB2] = interfaceM(self)

    v=self.res.v(~isnan(self.res.v)); % Velocity
    a=self.res.a; % get acceleration
    dt =diff(self.res.t); %get dt
    t = [0; cumsum(dt)];
    aux = self.paramVariable.p_aux_w;
    stSOC = self.stSOC;
    T_1 = self.res.T_1;
    T_b = self.res.T_b;
    
    %Motorrevs Motor 1
    n_1 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_1; % Motor 1 speed  in [1/s]
    
    
    if self.splitGbMap ==1
        Slack_GB=self.res.Sl_loss_gb;
        if self.twoMotors == 1
            Slack_GB2 = self.res.Sl_loss_gb_2;
        else
            Slack_GB2=[];
        end
    else
        Slack_GB=[];
        Slack_GB2=[];
    end
    
    if self.twoMotors == 0
        n_2=[];
        T_2 = [];
    elseif self.twoMotors == 1
        %Motorrevs Motor 2
        Cl = [];
        n_2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr2; % Motor 2 speed  in [1/s]
        T_2 = self.res.T_2;
    end
    
    if self.twoGears==0
        Cl = [];
    elseif self.twoGears == 1
        Cl = self.res.Cl;
        %Cl =reshape(repmat(Cl,uint8(1),uint8(self.n/self.n_gear))',self.n,1);
        n_1_2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_2;
        n_1 = (2-Cl) .* n_1+ (Cl-1) .*  n_1_2;  % Actual Motor speed as function of clutch position
    end
    
end

function [v, a, dt, t, aux, stSOC,n_1,n_2, T_1, T_2, T_b, Cl,Slack_GB,Slack_GB2] = interfaceCycle(self,v,t,aux,stSOC)

%v=self.dm.v(~isnan(self.dm.v)); % Velocity

%t = dm.t;
dt =diff(t);
a=diff(v)./dt; % get acceleration

T_1 = ones(size(a));
T_b =zeros(size(a));
%Motorrevs Motor 1
n_1 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_1; % Motor 1 speed  in [1/s]

% Slack GB
if self.splitGbMap ==1
    Slack_GB=ones(size(a));
    if self.twoMotors == 1
        Slack_GB2= ones(size(a));
    else
        Slack_GB2=[];
    end
else
    Slack_GB=[];
    Slack_GB2=[];
end

if self.twoMotors == 0
    n_2=[];
    T_2=[];
end
if self.twoMotors == 1
    %Motorrevs Motor 2
    n_2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr2; % Motor 2 speed  in [1/s]
    T_2 = zeros(size(a));
end
    
    if self.twoGears==0
        Cl = [];
    elseif self.twoGears == 1
        Cl =ones(size(a));
        n_1_2 = v(1:end-1) / (2*pi * self.veh.r) * self.veh.i_gr1_2;
        n_1 = (2-Cl) .* n_1+ (Cl-1) .*  n_1_2;  % Actual Motor speed as function of clutch position
    end


end

function [T_loss, P_loss] = calcGearboxLossesMap(gearbox,n,T)

xdgb =  gearbox.data.n_rps(:,1);
ydgb =  gearbox.data.T_Nm(1,:);
[xdgb,ydgb]=ndgrid(xdgb,ydgb);

GRIDINT_GB=griddedInterpolant(xdgb ,  ydgb, gearbox.losses.T_loss,'linear','linear' );

T_loss = GRIDINT_GB(n,abs(T));
P_loss = T_loss .* n * 2 *pi;

end

function [T_loss_mot, P_loss_mot_mech] = calcMechMotorLossesMap(motor,omega,T)

xd =  motor.data.n_radps(:,1);
yd =  motor.data.T_Nm(1,:);
[xd,yd]=ndgrid(xd,yd);

P_loss_grid =  motor.losses.mech_W;
P_loss_grid = fillmissing(P_loss_grid,'linear',2,'EndValues','nearest');
GRIDINT=griddedInterpolant(xd,yd,  P_loss_grid,'linear','linear' );

P_loss_mot_mech=GRIDINT(omega, abs(T));

T_loss_mot = P_loss_mot_mech./omega;
T_loss_mot(isnan(T_loss_mot))=0;
T_loss_mot(isinf(T_loss_mot))=0;
end

function [P_mot,P_loss] = calcElMotorLossesMap(motor,omega,T)

xd = motor.data.n_radps(:,1);
yd = motor.data.T_Nm(1,:);
[xd,yd]=ndgrid(xd,yd);

P_loss_grid = motor.losses.inverter_W+motor.losses.iron_W+motor.losses.stator_W;


P_loss_grid = fillmissing(P_loss_grid,'linear',2,'EndValues','nearest');

GRIDINT=griddedInterpolant(xd,yd,  P_loss_grid,'linear','linear' );

P_loss=GRIDINT(omega, abs(T));

P_mot = P_loss + omega.*T;
end

function plotLosses(Losses,twoMotors)

figure
hold on

[ TumColors ] = HelpFun.tumColors(  );

%% Prepare Plot bar
y = [Losses.Energy.E_loss_WtD(end) Losses.Energy.Brake(end)  Losses.Energy.E_loss_gb1(end) Losses.Energy.E_loss_mot1(end) Losses.Energy.E_aux(end) Losses.Energy.E_loss_bat(end)];


if twoMotors == 1
    y = [Losses.Energy.E_loss_WtD(end) Losses.Energy.Brake(end)  Losses.Energy.E_loss_gb1(end)+Losses.Energy.E_loss_gb2(end) Losses.Energy.E_loss_mot1(end)+Losses.Energy.E_loss_mot2(end) Losses.Energy.E_aux(end) Losses.Energy.E_loss_bat(end)];
    
end

%kWh
%y = y./3600000;

%% Plot bar

ba=bar([1;nan], [y; nan(size(y))], 'stacked','FaceColor','flat');
%set(gca,'xticklabel',{'urban','inter-city','highway'});
legend( 'Wheel-to-Distance-Losses','Braking-Losses','Gearbox Losses','Motor Losses','Auxilary','Battery Losses','Location','south')
ylabel('Energy in kWh')     
set(gca, 'xticklabel', {' '})
ba(1).CData = TumColors.primaryBlue;
ba(2).CData = TumColors.secondaryGrey;
ba(3).CData = TumColors.accentGreen;
ba(4).CData = TumColors.accentLightBlue;
ba(5).CData = TumColors.accentOrange;
ba(6).CData = TumColors.extended1;

end





function [P_mot,P_loss_mot_el, P_loss_mot_mech, P_loss_gb,T_mot_fin,n_mot] = quasistaticAxleSim(veh, motor, gearbox,gearratio, F_wheel,v)
%% Quasistatic Simulation

T_wheel = F_wheel *  veh.r;
T_mot = T_wheel ./ gearratio;
n_mot = v(1:end-1)/(2 *  pi *  veh.r) .*gearratio;

%%%%%%%%%%%%%%%%%%%%%%%%%GB
% Loop to fit gearboxlosses
[T_loss_init, ~] = calcGearboxLossesMap(gearbox,n_mot,T_mot);
T_mot_old=T_mot;
T_mot_fin=T_mot_old+T_loss_init;
[T_loss, ~] = calcGearboxLossesMap(gearbox,n_mot,T_mot_fin);

while sum(abs(T_mot_old-T_mot_fin))^2 > 0.1
    [T_loss, ~] = calcGearboxLossesMap(gearbox,n_mot,T_mot_fin);
    T_mot_old =T_mot_fin;
    T_mot_fin = T_mot+T_loss;
end
P_loss_gb = T_loss .* n_mot *2 *pi;

%%%%%%%%%%%%%%%%%%%%%%%%Mot
[T_loss_mot_mech, ~] = calcMechMotorLossesMap(motor,n_mot*2*pi,T_mot_fin);
T_mot_fin= T_mot_fin + T_loss_mot_mech;

P_loss_mot_mech = T_loss_mot_mech .* n_mot *2 *pi;

[P_mot,P_loss_mot_el] =  calcElMotorLossesMap(motor,n_mot*2*pi,T_mot_fin);


end

function [Res] = genPolyFun(info,x,y)

Res = info.coeffs(1) .* x.^info.Exponents(1,1)  .* y.^info.Exponents(1,2);

for i =2:length(info.coeffs)
    Res = Res ...
        +info.coeffs(i) .* x.^info.Exponents(i,1)  .* y.^info.Exponents(i,2);
end
end


function [P_bat,P_loss,SOC] = calcBatteryLosses(self, P_bat_in,stSOC,dt)

SOC = [stSOC; nan(size(P_bat_in))];
P_loss=nan(size(P_bat_in));
P_bat=nan(size(P_bat_in));

for i = 1 :length(P_bat_in)
    

    V_cell =  interp1(self.veh.bat.bat_cell.SOC,self.veh.bat.bat_cell.OCV,SOC(i)/100);
    %
    P_cell = V_cell.^2 / (2*self.veh.bat.bat_cell.R_i_bat) - ...
        V_cell.* sqrt((V_cell.^2 - 4 * (P_bat_in(i)/self.veh.bat.numcelser/self.veh.bat.numcellpa)...
        *self.veh.bat.bat_cell.R_i_bat)./(4*...
        self.veh.bat.bat_cell.R_i_bat^2));
    %
    if ~isreal(P_cell)
        warning('P_cell not real!')
        P_cell = real(P_cell)
    end
    
    P_bat(i) = P_cell * self.veh.bat.numcelser * self.veh.bat.numcellpa;
    P_loss(i)=P_bat(i)-P_bat_in(i);
    
    I_cell = P_cell/V_cell;
    Q_cell = I_cell .* dt(i);
    dSOC = 100*Q_cell./(self.veh.bat.bat_cell.Ah*60*60);
    SOC(i+1)=SOC(i)-dSOC;
end
end
