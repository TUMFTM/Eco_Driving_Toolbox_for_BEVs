function [res] = extractSolution (self)
% Designed by: Alexander Koch (FTM, Technical University of Munich)
%-------------
% Created on: 2019-2022
% ------------
% Version: Matlab2021a
%-------------
% Description: Cuts long solution vector into smaller pieces and names
% them.
% ------------
% Input:    - self: Optimizer Object
% ------------
% Output: - res: struct. solution of optiization
% ------------


%% Extract solution
x=full(self.solution.x);

if self.indVar == 't'
    
    res.T_1=x(1:self.n);
    res.T_b=x(1*self.n+1:2*self.n);
    res.a=x(2*self.n+1:3*self.n);
    res.v=x(3*self.n+1:3*self.n+self.m);
    res.s=x(3*self.n+self.m+1:3*self.n+2*self.m);
    res.P_b=x(3*self.n+2*self.m+1:4*self.n+2*self.m);
    res.SOC=x(4*self.n+2*self.m+1:4*self.n+3*self.m);
    res.I_b=x(4*self.n+3*self.m+1:5*self.n+3*self.m);
    n_end = 5*self.n+3*self.m;
    
    
    if self.twoGears == 1
        res.Cl = x(n_end+1:n_end+self.nGear);
        n_end = n_end+self.nGear;
    end
    
    if self.twoMotors == 1
        res.T_2 = x(n_end+1:n_end+self.n);
        n_end=n_end+self.n;
    end
    if self.splitMotMap ==1
        res.Sl_loss_mot = x(n_end+1:n_end+self.n);
        n_end = n_end+self.n;
    end
    if self.splitGbMap == 1
        res.Sl_loss_gb = x(n_end+1:n_end+self.n);
        n_end = n_end+self.n;
    end
    
    
    if self.twoMotors == 1
        if self.splitMotMap ==1
            res.Sl_loss_mot_2 = x(n_end+1:n_end+self.n);
            n_end = n_end+self.n;
        end
        if self.splitGbMap == 1
            res.Sl_loss_gb_2 = x(n_end+1:n_end+self.n);
            n_end = n_end+self.n;
        end
    end
    
elseif self.indVar == 's'
    res.T_1=x(1:self.n);
    res.T_b=x(1*self.n+1:2*self.n);
    res.a=x(2*self.n+1:3*self.n);
    res.v=x(3*self.n+1:3*self.n+self.m);
    res.t=x(3*self.n+self.m+1:3*self.n+2*self.m);
    res.P_b=x(3*self.n+2*self.m+1:4*self.n+2*self.m);
    res.SOC=x(4*self.n+2*self.m+1:4*self.n+3*self.m);
    n_end = 4*self.n+3*self.m;
    
    
    if self.twoGears == 1
        res.Cl = x(n_end+1:n_end+self.nGear);
        n_end = n_end+self.nGear;
    end
    
    if self.twoMotors == 1
        res.T_2 = x(n_end+1:n_end+self.n);
        n_end=n_end+self.n;
    end
    
end

%% Objective
res.obj = full(self.solution.f);


end







