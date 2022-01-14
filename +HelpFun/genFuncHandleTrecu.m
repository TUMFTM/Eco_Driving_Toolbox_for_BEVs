function [han] = genFuncHandleTrecu(scaling, x_step, y_0, y_step)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Create function handle for function handling on different
% PCs
% ------------
% Input:    - see Helpfun.atanstep
% ------------
% Output:    - Function Handle
% ------------
han = @(n_radps) genFuncHandleTrecu(scaling, x_step, y_0, y_step, n_radps);

end