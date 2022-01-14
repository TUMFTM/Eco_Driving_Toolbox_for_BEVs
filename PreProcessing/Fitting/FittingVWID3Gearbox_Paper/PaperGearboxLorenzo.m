% Skript to fit gearboxes of VW ID3 and ASM for paper
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2021
% ------------
% Version: Matlab2021a
%-------------

clear

%Path
addpath(genpath('PreProcessing/Fitting/FittingTool'));


for i =1:2
    
    %Read Data
    if i == 1
        load('DataExtern\PowertrainComponents\Gearbox\VWID3\gearbox_ID3_11.mat');
        i_trans = 11.53;
        savename = 'GB_ID3_Paper';
    elseif i == 2
        load('DataExtern\PowertrainComponents\Gearbox\VWID3\gearbox_losses_ASM_i_8.mat');
        i_trans = 8;
        savename = 'GB_ID3_ASM_Paper';
    end
    
    % Fill first line by interpolation
    gearbox_losses.T_tot(:,1) = gearbox_losses.T_tot(:,2) +  (gearbox_losses.T_tot(:,2)- gearbox_losses.T_tot(:,3));
    
    % Create gearbox struct
    gearbox.data.n_rps = flip(gearbox_losses.n_mot/60)';
     gearbox.data.n_radps =  gearbox.data.n_rps*2*pi;
    gearbox.data.T_Nm = flip(gearbox_losses.T_mot)';
    gearbox.losses.T_loss = flip(gearbox_losses.T_tot)';
    gearbox.data.i_gr = i_trans;

    
    % Split FIT
    PARA.polyX = 2;
    PARA.polyY = 3;
    PARA.Method = 'min SRE';
    
    % Prepare vectors   
    x=reshape(gearbox.data.n_radps,1,[]);
    y=reshape(gearbox.data.T_Nm,1,[]);
    z=reshape(gearbox.losses.T_loss,1,[]);
    
    y(isnan(x))=[];
    z(isnan(x))=[];
    x(isnan(x))=[];
    
    y(isnan(z))=[];
    x(isnan(z))=[];
    z(isnan(z))=[];
    
    x_ineq = x(1:100:end);
    y_ineq = -y(1:100:end);
    z_ineq = z(1:100:end);
    
    % Fitting
    [~,info] = ...
        HelpFun.fittingEDT(x,y,z,PARA,[],[],[],x_ineq,y_ineq,z_ineq);
    
    % Save results
    gearbox.fit_sp = info;
    
    % Evaluate
    figure
    hold on
    mesh(gearbox.data.n_radps,gearbox.data.T_Nm, gearbox.fit_sp.fitFun(gearbox.data.n_radps,gearbox.data.T_Nm))
    scatter3(reshape(gearbox.data.n_radps,1,[]),...
        reshape(gearbox.data.T_Nm,1,[]),...
        reshape(gearbox.losses.T_loss,1,[]),3,'filled')
    mesh(gearbox.data.n_radps,-gearbox.data.T_Nm, gearbox.fit_sp.fitFun(gearbox.data.n_radps,-gearbox.data.T_Nm))
    title('Gearbox loss map')
    xlabel('Motor speed in radpm')
    ylabel('Input torque in Nm')
    zlabel('Gearbox losses in Nm')
    legend('Fit','Data')
    view(40,35)
    
    
    % Continous FIT
    % options
    PARA.polyX = 2;
    PARA.polyY = 6;
    PARA.Method = 'min SRE';
    
    % Prepare vectors
    x= [x x];
    y=[y -y];
    z=[z z];
    
    % Fitting
    [fitFun,info] = ...
        HelpFun.fittingEDT(x,y,z,PARA);
    
    % Save results
    gearbox.fit_co = info;
    gearbox.fit_co.fitFun = fitFun;
    
    % Evaluate
    figure
    hold on
    mesh(gearbox.data.n_radps,gearbox.data.T_Nm, gearbox.fit_co.fitFun(gearbox.data.n_radps,gearbox.data.T_Nm))
    scatter3(reshape(gearbox.data.n_radps,1,[]),...
        reshape(gearbox.data.T_Nm,1,[]),...
        reshape(gearbox.losses.T_loss,1,[]),3,'filled')
    mesh(gearbox.data.n_radps,-gearbox.data.T_Nm, gearbox.fit_co.fitFun(gearbox.data.n_radps,-gearbox.data.T_Nm))
    title('Gearbox loss map')
    xlabel('Motor speed in radpm')
    ylabel('Input torque in Nm')
    zlabel('Gearbox losses in Nm')
    legend('Fit','Data')
    view(40,35)
    
    
    %% Save mat-file
    cd('Data/PowertrainFitData/Gearbox')
    save(savename,'gearbox');
    cd('../../..');
    
end

