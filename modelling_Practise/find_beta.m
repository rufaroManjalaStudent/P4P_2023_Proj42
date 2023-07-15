function [Cp]  = find_beta(beta, lambda) 

Cp = 0.5176*(116/(lambda+0.08*beta) - 4.06/(1+beta^3)-0.4*beta-5)*exp((-21/(lambda+0.08*beta)+0.735/(1+beta^3)+0.0068*lambda));

end