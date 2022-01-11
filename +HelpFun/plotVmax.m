function plotVmax(dm) 
% Designed by: Alexander Koch(FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Plots spatial v-max based on atan functions
% ------------
% Input:    - dm, struct driving mission
% ------------

s = dm.lv_s();


vmax=dm.v_max;
vec_s = dm.v_max_s;

scaling = 150;
 funVmax = vmax(1);

for i = 2:length(vmax)
    
    v_change = vmax(i)-vmax(i-1);
     s_change = vec_s(i);
    funVmax = funVmax + ...
         v_change *  (0.5+  atan(scaling*(s-s_change))/pi);
end


plot(s,funVmax,'b','LineWidth',2)

end