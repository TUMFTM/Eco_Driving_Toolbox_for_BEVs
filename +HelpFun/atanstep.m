function y = atanstep(scaling, x_step, y_0, y_step, x)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Calculates a atan step function
% ------------



         y = y_0 + y_step *  (0.5 +  atan(scaling*(x-x_step))/pi);
  
end