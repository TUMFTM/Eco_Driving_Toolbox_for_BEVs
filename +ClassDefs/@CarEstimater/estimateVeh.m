function [est] = estimateVeh(self,time)
   %estimateVeh: Method to estimate movement of leading vehicle.
   % Author:   Alexander Koch, Ph.D. Candidate, TU Munich, Institute for Automotive Technologie
   % Date:     11.01.22

if strcmp(self.predType,'V2V')
    est.t = self.horizon + time;
    est.v = interp1(self.dm.t, self.dm.lv_v,est.t,'previous','extrap');
    est.s = interp1(self.dm.t, self.dm.lv_s,est.t,'previous','extrap');
    
elseif strcmp(self.predType,'a')
    
    est.t = self.horizon + time;
    
    v = interp1(self.dm.t, self.dm.lv_v,est.t(1),'linear','extrap');
 
    cur_v = v(1);
    cur_a = (v(1)-self.vLast)/(time-self.tLast);
    cur_s =  interp1(self.dm.t, self.dm.lv_s,time);
   
    vPred = cur_v + cur_a *self.horizon;
    
    sPred = cur_s + cumsum(vPred .* [0; diff(self.horizon)]);
    
    if cur_a > 0
        v_max = interp1(self.dm.v_max_s, self.dm.v_max,sPred,'previous','extrap');
        vPred = min(vPred,v_max);
    elseif cur_a < 0
        vPred = max(vPred,0);
    end
        
    sPred = cur_s + cumsum(vPred .* [0; diff(self.horizon)]);
    
    est.v = vPred;
    est.s = sPred;
    
    self.vLast = cur_v;
    self.tLast = time;
    

self.est=est;

end