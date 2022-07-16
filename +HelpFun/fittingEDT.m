function [fitFun,info] = ...
    fittingEDT(xM_use,yM_use,MAPM_use,PARA,x_eq,y_eq,z_eq,x_ineq,y_ineq,z_ineq)
% Designed by: Tim Bürchner, Xucheng Duan, Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Fittes 3-D contours like motor loss maps, etc. for the
% Eco-Driving-Toolbox based on least squares or relative least squares.
% ------------
% PARA options:
%              - PARA.Weights: vector with weights for fitting
%              - PARA.Method: 'min SE' or 'min SRE', min SRE for relative least
%              squares
%              - PARA.OnlyPositiveCoeff: 0/1, if 1: fitting coeiffcients must be
%              positive
%              - PARA.IgnoreExp: '{pxy}', coeffcient p_x_y is ignored
%              - PARA.FlipIneq: 0/1, if 1: A_ineq = -A_ineq;
% ------------
% Input:    - xM_use: x-values
%              - yM_use: y-values
%              - MAPM_use: z-values
%              - PARA: struct with Parameters
%              - x_eq: x-values for equality constraints
%              - y_eq: y-values for equality constraints
%              - z_eq: z-values for equality constraints
%              - x_ineq: x-values for inequality constraints
%              - y_ineq: y-values for inequality constraints
%              - z_ineq: z-values for inequality constraints
% ------------
% Output:    - fitFun: symbolic fitting function fitFun(x,y)
%                 - info: all required fitting information for
%                 eco-driving-algorithm
% ------------


% Highest polynom order
polyMAX = max(PARA.polyX, PARA.polyY);
% Initialisation of number of coefficients
N_p = 0;
% Saving of exponents
Exponents = [];


% Calculate number of coefficients and name
for p_y = 0:PARA.polyY
    for p_x = 0:PARA.polyX
        if (p_x+p_y) <= polyMAX
            N_p = N_p + 1;
            name{N_p} = ['p' num2str(p_x) num2str(p_y)];
            Exponents = [Exponents; p_x p_y];
        end
    end
end

%Remove exponents which should be ignored
if isfield(PARA,'IgnoreExp')
    for ign = 1:length(PARA.IgnoreExp)
        idx_ign = strcmp(PARA.IgnoreExp{ign},name);
        idx_ign = [1:1:length(idx_ign)]*idx_ign';
        Exponents(idx_ign,:)=[];
        N_p = N_p - 1;
        name{ign} = [];
    end
end

% Save number of exponents and the exponents
info.Ncoeffs = N_p;
info.Exponents = Exponents;

% Build matrices for quadratic optmization
f_allM = zeros(N_p,1);
F_allM = zeros(N_p,N_p);

xM_use = xM_use(~isnan(MAPM_use));
yM_use = yM_use(~isnan(MAPM_use));
MAPM_use = MAPM_use(~isnan(MAPM_use));

if isfield(PARA,'Weights')
    WtM_use = PARA.Weights(~isnan(MAPM_use));
else
    WtM_use = ones(size(MAPM_use));
end

%Determine Method
if ~isfield(PARA,'Method')
    warning('default Fitting Method (min SE) is used.')
    PARA.Method = 'minSE';
end

%% Matrix System for x y z
for i = 1:length(xM_use)
    f_now = zeros(N_p,1);
    for j = 1:N_p
        f_now(j) = ...
            (xM_use(i).^(Exponents(j,1)) .* yM_use(i).^(Exponents(j,2)))';
    end
    
    if strcmp(PARA.Method,'min SE') % minimum squared error
        f_allM = f_allM - f_now * MAPM_use(i) * WtM_use(i);
        F_allM = F_allM + f_now * f_now' * WtM_use(i);
    elseif strcmp(PARA.Method,'min SRE') % minimum squared relative error
        if MAPM_use(i)~=0
            f_allM = f_allM - f_now / MAPM_use(i) * WtM_use(i);
            F_allM = F_allM + f_now * f_now' / (MAPM_use(i)^2) * WtM_use(i);
        end
    else
        error('invalid Fitting Method!')
    end
    
end




% %TEST
% %% Matrix System for x y z
% T_A = [];
% T_b=[];
% for i = 1:length(xM_use)
%     f_now = zeros(N_p,1);
%     for j = 1:N_p
%         f_now(j) = ...
%             (xM_use(i).^(Exponents(j,1)) .* yM_use(i).^(Exponents(j,2)))';
%     end
%
%     if strcmp(PARA.Method,'min SE') % minimum squared error
%                  T_A = [T_A;f_now'];
%                     T_b = [T_b;MAPM_use(i)];
%     elseif strcmp(PARA.Method,'min SRE') % minimum squared relative error
%         if MAPM_use(i)~=0
%                  T_A = [T_A;(f_now./MAPM_use(i))'];
%                  T_b = [T_b; 1];
%         end
%     end
%
%
%
% end
%
% T_b = -1*T_A'*T_b;
% T_A = T_A'*T_A;
%%%% TEST --> is the same


%% Equality Constraint
if nargin >= 7
    A_eq = [];
    b_eq = [];
    
    %% Matrix System for x_eq y_eq z_eq
    for i = 1:length(x_eq)
        
        for j = 1:N_p
            A_eq(i,j) = (x_eq(i).^(Exponents(j,1)) .* y_eq(i).^(Exponents(j,2)))';
        end
        b_eq(i) = z_eq(i);
    end
    
else
    A_eq = [];
    b_eq = [];
end



%% Inequ Const.
if nargin >= 9
    for i = 1:length(x_ineq)
        for j = 1:N_p
            A_ineq(i,j) =  (x_ineq(i).^(Exponents(j,1)) .* y_ineq(i).^(Exponents(j,2)))';
        end
        b_ineq(i) = z_ineq(i);
    end
else
    A_ineq = [];
    b_ineq = [];
end

%% For Validation fitting
if isfield(PARA,'OnlyPositiveCoeff')
    if PARA.OnlyPositiveCoeff==1
        A_ineq = [A_ineq;-1*eye(N_p)];
        b_ineq = [b_ineq; zeros(N_p,1)];
    end
end


if isfield(PARA,'FlipIneq')
    if PARA.FlipIneq==1
        A_ineq = -A_ineq;
    end
end


% Fitting with fmincon
options = optimoptions('quadprog','Display','final');
options.OptimalityTolerance=10^-10;
options.MaxIterations=5000000;
[solutionM, ~] = quadprog(F_allM,f_allM,A_ineq,b_ineq,A_eq,b_eq,[],[],[],options);


% Save solution
%fitFun =@(x,y) HelpFun.CalcPolyValue(x,y,solutionM,Exponents);
info.coeffs = solutionM;
fitFun =@(x,y) +HelpFun.genPolyFun(info,x,y);
info.fitFun =  fitFun;
info.rmse = sqrt(     sum(    (fitFun(xM_use,yM_use)-MAPM_use).^2.   )    /numel(xM_use)            );
info.rmsre = sqrt(    1/ numel(xM_use(MAPM_use~=0))    *  sum(    ((fitFun(xM_use(MAPM_use~=0),yM_use(MAPM_use~=0))-MAPM_use(MAPM_use~=0))./MAPM_use(MAPM_use~=0)).^2.   )              );


end

