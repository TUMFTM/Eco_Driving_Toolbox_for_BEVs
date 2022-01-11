function [Value] = CalcPolyValue(x,y,coeff,Exponents)
% Designed by: Tim Bürchner(FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Calculates the zValues based on x and y Values and a
% polynomial function represented with coefficients and exponents.
% ------------
% Input:    - x: xValues
%              - y: yValues
%              - coeff: coefficients of expontents
%              - Exponents: exponents
% ------------

% Dimensions of x
dim_x = ndims(x);
% Size of x
s_x = size(x);
% Number of exponents
N_exp = size(Exponents,1);

% Expand x /y in one dimension Exponents for the values
x = repmat(x,[ones(1,dim_x),N_exp]);
y = repmat(y,[ones(1,dim_x),N_exp]);
Exponents_x = repmat(reshape(Exponents(:,1),[ones(1,dim_x),N_exp]),[s_x,1]);
Exponents_y = repmat(reshape(Exponents(:,2),[ones(1,dim_x),N_exp]),[s_x,1]);
coeff = repmat(reshape(coeff,[ones(1,dim_x),N_exp]),[s_x,1]);

% Calculate values
Value = sum(coeff .* x.^Exponents_x .* y.^Exponents_y, dim_x+1);

end