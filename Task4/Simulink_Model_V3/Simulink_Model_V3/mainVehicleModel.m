%% Main script Vehicle Model

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ % 
% Authors: Francesco Biral: francesco.biral@unitn.it  %
%          &                                          %
%          Luca De Pascali: luca.depascali@unitn.it   %
%                                                     %
% Last release: May, 6th 2018                         %
% Revised:      June, 9th 2020                        %  
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ % 

% This script initializes the parameters and starts the simulation of the
% Simulink model 'FullVehicleModel_ABS_ESP.slx'. The model simulates the
% dynamics of a full vehicle model, namely:
%   - longitudinal speed (u)
%   - lateral speed  (v)
%   - yaw rate (Omega)
%   - roll angle and roll rate (phi and phi_dot)
%   - wheel lateral slip (alpha_i),     i = rr,rl,fr,fl
%   - wheel longitudinal slip (kappa_i) i = rr,rl,fr,fl
%   - wheel vertical load (Fz_i)        i = rr,rl,fr,fl
%   - wheel angular speed (omega_i)     i = rr,rl,fr,fl

% The tire-road contact forces are computed with the Pacejka's magic 
% formula, whose coefficients and equations are contained in file
% 'pacejka_model.m'

% You need to put this script in a dedicated folder together with the
% following files:
%   - 'FullVehicleModel_V3_sfun.m'
%   - 'FullVehicleModel_ABS_ESP.slx'
%   - 'low_speed_slip.m'
%   - 'pacejka_model.m'
%   - 'regSign.m'
%   - 'dataAnalysis.m'
%   - 'color.m'
%   - 'getVehicleDataStruct.m'
%   - 'getInitialConditionsDataStruct.m'
%   - 'getAuxiliaryDataStruct.m'

%% Initialize
clear
close all
clc







% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

% Set defualt figure window as 'docked'
set(0,'DefaultFigureWindowStyle','docked');

fprintf(strcat('------ SIMULATION OF VEHICLE DYNAMICS---------------- \n'))
fprintf(strcat('    Double track 8dof model                           \n'))
fprintf(strcat('    Simulink model ready for ABS/ESP algorithm testing\n'))
fprintf(strcat('                    \n'))
fprintf(strcat('    University of Trento -  DII\n'))
fprintf(strcat('    MSc Course Dynamic and Control of Vehicle and Robot\n'))
fprintf(strcat('------------------------------------------------------ \n'))
fprintf(strcat('                    \n\n\n'))


%% Load vehicle and auxiliary data
fprintf(strcat('Loading vehicle data \n'))

% You can change the data in getVehicleDataStruct.m according to what you
% need. Everything is parameterized.
% Tyre data are contained in loadPacejkaParam.m
% Be careful if you want to change tyre data.





vehicle_data   = getVehicleDataStruct();
auxiliary_data = getAuxiliaryDataStruct();

Tb_init     = 0;
lambda_init = 0;

lambda_min = -0.08;
lambda_max = -0.055;
Fb_min = -2350;
Fb_max = -1850;

Tb_min = Fb_min*vehicle_data.rear_wheel.Rr;
Tb_max = Fb_max*vehicle_data.rear_wheel.Rr;

%k = 5e3; % [Nm/s]

disp('Initialised!')
%% Select Maneuvre
% In this section you can select one of the three maneuvers on which you
% are supposed to test the performances of your controllers:

% Start from different initial speed V_init and assess the performances of
% your controller at different speeds

V_init = 50; % 50; % [km/h] <------- Change Speed to assess your controller at different speed
init_conditions_data = getInitialConditionsDataStruct( V_init );

% ----------------------------------------------------------------------- %
% FULL BRAKE MANEUVER                                                     %
% ----------------------------------------------------------------------- %
% This maneuver allows you to assess the performances of the ABS control:
% the vehicle starts with initial speed V_init and at t_brake=3s the driver
% pushes down the brake pedal commanding a full brake (and inducing wheel lock)

% The aim of ABS control is that to reduce the stopping distance by acting
% on the braking torque. You can choose to implement ABS for a
% brake-by-wire system (brake torque continously varying) or for standard
% Hydraulic Actuated Brakes (HAB), characterized by discrete dynamics.

% % % % % % % % % % % % % % % %
brake_maneuver_flag = true;  % <------- Put this flag equal to true to select full brake maneuver 
% % % % % % % % % % % % % % % %

% Simulation lasts t_f = 10s;
if brake_maneuver_flag
  t_f           = 7; % [s]
  t_brake       = 3; % [s]
  maneuver_flag = 1;
  str = ' brake maneuver'; 
end

% Start from different initial speed V_init and compute the braking time and
% distance with and without ABS control

% ----------------------------------------------------------------------- %
% DOUBLE LANE CHANGE MANEUVER %
% ----------------------------------------------------------------------- %
% This maneuver allows you to assess the performances of the ESP control:
% the vehicle starts with initial speed V_init and the driver keeps the
% velocity constant and performs a double lane change maneuver at t=1s;

% % % % % % % % % % % % % % % % % % % % 
dblLaneChange_maneuver_flag = false;   % <------- Put this flag equal to true to select double lane change maneuver 
% % % % % % % % % % % % % % % % % % % % 

% Simulation lasts t_f = 10s;
if dblLaneChange_maneuver_flag
  t_f           = 5; % [s]
  t_brake       = t_f+1; % No brake for this maneuvre
  maneuver_flag = 2;
  str = ' double lane change maneuver'; 
end
if dblLaneChange_maneuver_flag && brake_maneuver_flag && ~ConstCornerANDBrake_maneuver_flag
  warning('Two maneuvers selected. The simulation will perform only Double Lane Change Maneuver.')
end
% Start from different initial speed V_init and report the improvements
% obtained with your ESP controller

% ----------------------------------------------------------------------- %
% CONSTANT CORNERING AND BRAKE MANEUVER %
% ----------------------------------------------------------------------- %
% This maneuver allows you to assess the performances of the ESP and ABS
% control: the vehicle performs a constant velocity cornering maneuver and
% brakes while cornering after 10s

% % % % % % % % % % % % % % % % % % % % % %
ConstCornerANDBrake_maneuver_flag = false; % <------- Put this flag equal to true to select constant cornering and brake maneuver 
% % % % % % % % % % % % % % % % % % % % % %

% Simulation lasts t_f = 10s;
if ConstCornerANDBrake_maneuver_flag
  t_f           = 30; % [s]
  t_brake       = 31; % [s]
  maneuver_flag = 3;
  str = ' constant speed cornering and brake maneuver';
end

if (dblLaneChange_maneuver_flag && brake_maneuver_flag && ConstCornerANDBrake_maneuver_flag) || (dblLaneChange_maneuver_flag && ConstCornerANDBrake_maneuver_flag) || (brake_maneuver_flag && ConstCornerANDBrake_maneuver_flag)
  warning('Two or more maneuvers selected. The simulation will perform only Cornering and Brake Maneuver.')
end

% Start from different initial speed V_init and report the improvements
% obtained with your ESP controller


% ----------------------------------------------------------------------- %
% STEADY STATE CORNERING MANEUVER       %
% ----------------------------------------------------------------------- %
% This maneuver allows you to assess the steering behaviour of the vehicle.
% 

% % % % % % % % % % % % % % % % % % % % % %
SteadyStateCorner_flag = false; % <------- Put this flag equal to true to select constant cornering and brake maneuver 
% % % % % % % % % % % % % % % % % % % % % %

% Simulation lasts t_f = 10s;
if SteadyStateCorner_flag
  maneuver_flag  = 4;
  t_f            = 110; %180; % [s]
  t_brake        = t_f+1; % [s]
  curv_akermann  = 1/50.;  %[1/m]
  acc_US_test    = 0.1; %[m/s^2] <------- acceleration to have speed increment (should be low)
  Tstart_US_test = 6 ;  %[s]     <------- when test start
  str = ' Quasi steady state cornering test';
end

if (dblLaneChange_maneuver_flag && brake_maneuver_flag && ConstCornerANDBrake_maneuver_flag) ...
    || (dblLaneChange_maneuver_flag && ConstCornerANDBrake_maneuver_flag) ...
    || (brake_maneuver_flag && ConstCornerANDBrake_maneuver_flag)
  warning('Two or more maneuvers selected. The simulation will perform only Cornering and Brake Maneuver.')
end

% Start from different initial speed V_init and report the improvements
% obtained with your ESP controller

%% Start Simulation
fprintf(strcat('Starting Simulation for ',str,'...\n'))

%changing simulation parameters
% paramNameValStruct.SimulationMode = 'accelerator'; %'rapid';
% paramNameValStruct.AbsTol         = '1e-5';
% model_sim = sim('FullVehicleModel_ABS_ESP',paramNameValStruct);%,'TimeOut',60,'Trace','siminfo');
model_sim = sim('FullVehicleModel_ABS_ESP');%,'TimeOut',60,'Trace','siminfo');
fprintf('Simulation completed\n')

%% Post-Processing
% Plot of the interesting quantities for the evaluation of vehicle behavior
% The data analysis does also a plot wityh an animation
% NOTE: press any key to start the animation
fprintf('Plotting results\n')
dataAnalysis;











