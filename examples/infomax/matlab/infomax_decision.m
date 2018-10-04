function decs = infomax_decision(weights,patts)
% Infomax decision function, using the sum of the absolute values of
% membrane potentials

result = weights*patts;
decs = sum(abs(result));