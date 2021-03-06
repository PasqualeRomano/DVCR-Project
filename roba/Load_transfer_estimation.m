clc; close all; clear variables
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');set(groot,'defaulttextinterpreter','latex');  
%% *DYNAMICS AND CONTROL OF VEHICLES AND ROBOTS*
%% Intro
% In this script the telemetry data from the P1 experimental vehicles are loaded. 
% This Matlab Live Script can be used as the structure for your project script. 
% 
% Click "Save as..." and save this script as .m in order to obtain a classic 
% Matlab script.
%% Load Vehicle Parameters
% The script loads the vehicle main parameters in the |vehicle| struct. See 
% the script |p1_parameters.m| for comments about parameters names and units.

p1_parameters
%% Load Datasets
% The following test are available.
% 
% *DATASET NAME DESCRIPTION*
% 
% RAMP_STEER_L Left-hand ramp steer 10 m/s
% 
% RAMP_STEER_R Right-hand ramp steer 10 m/s
% 
% SINE_STEER_IS Sine wave steering at increasing speeds
% 
% SP_100FT_CR_IS_CCW Steering pad 100 ft circle constant radius increasing speed 
% counter-clockwise
% 
% SP_100FT_CR_IS_CW Steering pad 100 ft circle constant radius increasing speed 
% clockwise
% 
% STRAIGHT_LINE_0 Straight line at constant speed
% 
% STRAIGHT_LINE_1 Straight line in one direction
% 
% STRAIGHT_LINE_2 Straight line in the opposite direction
% 
% STEP_STEER Step steer
% 
% The loaded variable is a struct cointaing the following telemetry signals:

% DATA			UNITS		DESCRIPTION
% time                  s               acquisition time
% yaw			rad		yaw angle
% yaw_rate		rad/s 		yaw rate at COM
% roll			rad		roll at COM
% roll_rate		rad/s 		roll rate at COM
% long_vel		m/s 		longitudinal velocity of the COM
% lat_vel		m/s 		lateral velocity of the COM
% axG			m/s^2		longitudinal acceleration of the COM
% ayG			m/s^2 		lateral acceleration of the COM
% body_slip		rad		vehicle side slip angle (at COM)
% omega_FL		rad/s 		wheel angular speed
% omega_FR		rad/s 		wheel angular speed
% omega_RL		rad/s 		wheel angular speed
% omega_RR		rad/s 		wheel angular speed
% front_slip_angle      rad		equivalent (single track) slip angle at front axle
% rear_slip_angle       rad		equivalent (single track) slip angle at rear axle
% Fx_FL			N		tyre longitudinal force
% Fy_FL			N		tyre lateral force
% Fz_FL			N		tyre vertical force
% Fx_FR			N		tyre longitudinal force
% Fy_FR			N		tyre lateral force
% Fz_FR			N		tyre vertical force
% x_pos			m		COM position x coordinate (from GPS)
% y_pos			m		COM position y coordinate (from GPS)
% z_pos			m		COM position z coordinate or altitude (from GPS)
% delta_L		rad		steering angle of the front left wheel
% delta_R		rad		steering angle of the front right wheel
% delta_HW 		rad		handwheel steering angle
%% Side slip estimation

load("Dataset/STRAIGHT_LINE_0.mat");
load("Dataset/STRAIGHT_LINE_1.mat");
load("Dataset/RAMP_STEER_L.mat");
load("Dataset/RAMP_STEER_R.mat");
load("Dataset/SP_100FT_CR_IS_CCW.mat");
load("Dataset/SP_100FT_CR_IS_CW.mat");
load("roll_rad.mat","R_dyn")
load("roll_height.mat","h_roll");

hG = vehicle.h_Gs + h_roll

DeltaFz_x_SL1 = vehicle.mass*STRAIGHT_LINE_1.axG*hG/vehicle.L;
DeltaFz_x_SL0 = vehicle.mass*STRAIGHT_LINE_0.axG*hG/vehicle.L;

DeltaFz_y_f_CCW = vehicle.mass*SP_100FT_CR_IS_CCW.ayG/vehicle.W.*(vehicle.Lf*h_roll/vehicle.L + vehicle.h_Gs/2);
DeltaFz_y_r_CCW = vehicle.mass*SP_100FT_CR_IS_CCW.ayG/vehicle.W.*(vehicle.Lr*h_roll/vehicle.L + vehicle.h_Gs/2);

Fz_f = vehicle.mass*9.81*vehicle.Lr/vehicle.L;
Fz_r= vehicle.mass*9.81-Fz_f;

Fz_fl_CCW = Fz_f/2 - DeltaFz_y_f_CCW;
Fz_fr_CCW = Fz_f/2 + DeltaFz_y_f_CCW;

Fz_rl_CCW = Fz_r/2 - DeltaFz_y_r_CCW;
Fz_rr_CCW = Fz_r/2 + DeltaFz_y_r_CCW;


figure
subplot(2,1,1)
plot(SP_100FT_CR_IS_CCW.time,SP_100FT_CR_IS_CCW.Fz_FL+SP_100FT_CR_IS_CCW.Fz_FR,'DisplayName','front axle vertical load')
hold on
plot(SP_100FT_CR_IS_CCW.time,Fz_fl_CCW,'DisplayName','front left tyre load ')
hold on
plot(SP_100FT_CR_IS_CCW.time,Fz_fr_CCW,'DisplayName','front right tyre load')
legend
xlabel('time (s)');ylabel('N');
title('SP_100FT_CR_IS_CCW - front');
subplot(2,1,2)
plot(SP_100FT_CR_IS_CCW.time,vehicle.mass*9.81-SP_100FT_CR_IS_CCW.Fz_FL-SP_100FT_CR_IS_CCW.Fz_FR,'DisplayName','rear axle vertical load')
hold on
plot(SP_100FT_CR_IS_CCW.time,Fz_rl_CCW,'DisplayName','rear left tyre load ')
hold on
plot(SP_100FT_CR_IS_CCW.time,Fz_rr_CCW,'DisplayName','rear right tyre load')
legend
xlabel('time (s)');ylabel('N');
title('SP_100FT_CR_IS_CCW - rear');