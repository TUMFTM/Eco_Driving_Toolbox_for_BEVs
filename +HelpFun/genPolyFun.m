function [res] = genPolyFun(info,x,y)
% Designed by: Alexander Koch(FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Generats polynomial function
% ------------
% Input:    - info: struct with coefficients and exponents
%              - x: xValues
%              - y: yValues
% ------------
% Output: - res: zValues
% ------------

res = info.coeffs(1) .* x.^info.Exponents(1,1)  .* y.^info.Exponents(1,2);

for i =2:length(info.coeffs)
    res = res ...
     +info.coeffs(i) .* x.^info.Exponents(i,1)  .* y.^info.Exponents(i,2);
end
end