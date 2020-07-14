function [A,B,C,D] = LinSingleTrackStateSpace(P)
%Linear Single Track Model: This functions returns the state space matrices
%A,B,C,D of the Linearized Single Track Model

% States    => [beta, Omega]'
% Inputs    => [delta]
% Outputs   => [beta, Omega]'

% Assign parameters

% C__f 
 C__f = P(1);
 
% C__r 
 C__r = P(2);
 
% J 
 J = P(3);
 
% V 
 V = P(4);
 
% l__f 
 l__f = P(5);
 
% l__r 
 l__r = P(6);
 
% m 
 m = P(7);

 
% Define matrices

A = [(-C__f - C__r) / m / V (-m * V ^ 2 - C__f * l__f + C__r * l__r) / m / V ^ 2; (-C__f * l__f + C__r * l__r) / J (-C__f * l__f ^ 2 - C__r * l__r ^ 2) / J / V;];
B = [C__f / V / m; C__f * l__f / J;];
C = [1 0; 0 1;];
D = [0; 0;];
end

