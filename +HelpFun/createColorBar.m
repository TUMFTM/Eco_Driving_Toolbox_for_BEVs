function [mymap] = createColorBar()
% Designed by: Alexander Koch(FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Generats ColorBar based on TUM-Colors
% ------------
% Output:    - mymap: colormap
% ------------


 [ TumColors ] = HelpFun.tumColors(  );

 %% Colorbar
 mycol{1} = [0 51 89] ./ 256;
 mycol{2} =  TumColors.primaryBlue;
 mycol{3} = TumColors.secondaryLightBlue;
 mycol{4} = [103 154 29] ./ 256;
 mycol{5} = [255 220 0] ./ 256;
 mycol{6} = [249 186 0] ./ 256;
 mycol{7} = [214 76 19] ./ 256;
 
  mymap = [];
 
 for i = 1:length(mycol)-1
    mymap = [mymap; ...
        linspace(mycol{i}(1), mycol{i+1}(1), 256)',linspace(mycol{i}(2), mycol{i+1}(2), 256)',linspace(mycol{i}(3), mycol{i+1}(3), 256)'];
 end

end