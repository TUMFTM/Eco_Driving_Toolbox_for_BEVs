function [linFit] = doLinFit(a00)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Create struct of linear fit for function handling
% ------------
% Input:    - a00: inclination
% ------------
% Output:    - linFit: Struct for eco-driving-toolbox
% ------------

linFit.Ncoeffs = 1;
linFit.Exponents = [1 0];
linFit.coeffs = a00;
linFit.fitFun =@(x,y) HelpFun.genPolyFun(motor.fit_lossMech,x,y);

end