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
%% Height of roll axis estimation

load("Dataset/STRAIGHT_LINE_0.mat")
load("Dataset/STRAIGHT_LINE_1.mat");
load("Dataset/RAMP_STEER_L.mat");
load("Dataset/RAMP_STEER_R.mat");
load("Dataset/SP_100FT_CR_IS_CCW.mat");
load("Dataset/SP_100FT_CR_IS_CW.mat");
load("roll_rad.mat","R_dyn")
%Check steering angles left-right
figure
subplot(2,1,1)
plot(smooth(RAMP_STEER_L.time,RAMP_STEER_L.roll*180/pi,500),'DisplayName','roll')
xlabel('time (s)');ylabel('deg');
hold on
plot(smooth(RAMP_STEER_L.time,RAMP_STEER_L.ayG,500),'DisplayName','ayG')
xlabel('time (s)');ylabel('m,/s^2');
title('RAMP_STEER_L');
legend

subplot(2,1,2)
plot(smooth(RAMP_STEER_R.time,RAMP_STEER_R.roll*180/pi,500),'DisplayName','roll')
hold on
plot(smooth(RAMP_STEER_R.time,RAMP_STEER_R.ayG,500),'DisplayName','ayG')
xlabel('time (s)');ylabel('deg');
title('RAMP_STEER_R');
legend
%%
%phi = vehicle.m*ayG*1/vehicle.k_roll  *hgs
ft = fittype('K.*x'); 

fit_kroll_RAMP_STEER_L = fit(smooth(RAMP_STEER_L.ayG,500)*vehicle.mass/vehicle.k_roll,smooth(RAMP_STEER_L.roll,500),ft,'StartPoint',vehicle.h_Gs);
k_roll_RAMP_L = fit_kroll_RAMP_STEER_L.K;
fit_kroll_RAMP_STEER_R = fit(smooth(RAMP_STEER_R.ayG,500)*vehicle.mass/vehicle.k_roll,smooth(RAMP_STEER_R.roll,500),ft,'StartPoint',vehicle.h_Gs);
k_roll_RAMP_R = fit_kroll_RAMP_STEER_R.K;
%using ccw,cw and extracting constanr acceleration regions
%TO DO : stiamo hgs usando tratti di CW e CCW be smart
