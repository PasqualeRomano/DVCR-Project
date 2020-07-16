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

%lambda__F
lambda__f = P(8);

%lambda__r
lambda__r  =P(9);
 
% Define matrices

A = [0 -1 0.1e1 / m / V 0.1e1 / m / V; 0 0 l__f / J -l__r / J; -C__f * V / lambda__f -C__f * V * l__f / lambda__f -V / lambda__f 0; -C__r * V / lambda__r C__r * V * l__r / lambda__f 0 -V / lambda__r;];
B = [0 0 C__f * V / lambda__f 0]';
C = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1;];
D = [0; 0; 0; 0];
end

