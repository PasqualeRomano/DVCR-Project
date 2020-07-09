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
%% Axle characteristics

load("Dataset/STRAIGHT_LINE_0.mat")
load("Dataset/STRAIGHT_LINE_1.mat");
load("Dataset/RAMP_STEER_L.mat");
load("Dataset/RAMP_STEER_R.mat");
load("Dataset/SP_100FT_CR_IS_CCW.mat");
load("Dataset/SP_100FT_CR_IS_CW.mat");
load("roll_rad.mat","R_dyn")
load("steer_ratio","tau_VW_mean");
%Check axles charateristics
figure
subplot(2,2,1)
scatter(abs(smooth(SP_100FT_CR_IS_CCW.ayG,1500)/9.81),abs(smooth(SP_100FT_CR_IS_CCW.front_slip_angle,1500)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(SP_100FT_CR_IS_CCW.ayG,1500)/9.81),abs(smooth(SP_100FT_CR_IS_CCW.rear_slip_angle,1500)),'DisplayName','rear axle charateristic')
xlabel('axG/g');ylabel('slip');
title('SP_100FT_CR_IS_CCW');
legend

subplot(2,2,2)
scatter(abs(smooth(SP_100FT_CR_IS_CW.ayG,800)/9.81),abs(smooth(SP_100FT_CR_IS_CW.front_slip_angle,800)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(SP_100FT_CR_IS_CW.ayG,800)/9.81),abs(smooth(SP_100FT_CR_IS_CW.rear_slip_angle,800)),'DisplayName','rear axle charateristic')
xlabel('axG/g');ylabel('slip');
title('SP_100FT_CR_IS_CW');
legend

subplot(2,2,3)
scatter(abs(smooth(RAMP_STEER_L.ayG,800)/9.81),abs(smooth(RAMP_STEER_L.front_slip_angle,800)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(RAMP_STEER_L.ayG,800)/9.81),abs(smooth(RAMP_STEER_L.rear_slip_angle,800)),'DisplayName','rear axle charateristic')
xlabel('axG/g');ylabel('slip');
title('RAMP_STEER_L');
legend

subplot(2,2,4)
scatter(abs(smooth(RAMP_STEER_R.ayG,800)/9.81),abs(smooth(RAMP_STEER_R.front_slip_angle,800)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(RAMP_STEER_R.ayG,800)/9.81),abs(smooth(RAMP_STEER_R.rear_slip_angle,800)),'DisplayName','rear axle charateristic')
xlabel('axG/g');ylabel('slip');
title('RAMP_STEER_R');
legend
%% Handling Diagram
figure
subplot(2,2,1)
scatter(smooth(abs(SP_100FT_CR_IS_CCW.ayG)/9.81,800),smooth(abs(SP_100FT_CR_IS_CCW.front_slip_angle-SP_100FT_CR_IS_CCW.rear_slip_angle),800),'DisplayName','Handling Diagram')
xlabel('ayG/g');ylabel('slip difference');
title('SP_100FT_CR_IS_CCW');
legend
subplot(2,2,2)
scatter(smooth(abs(SP_100FT_CR_IS_CW.ayG)/9.81,800),smooth(abs(SP_100FT_CR_IS_CW.front_slip_angle)-abs(SP_100FT_CR_IS_CW.rear_slip_angle),800),'DisplayName','Handling Diagram')
xlabel('ayG/g');ylabel('slip difference');
title('SP_100FT_CR_IS_CW');
legend
subplot(2,2,3)
scatter(smooth(abs(RAMP_STEER_R.ayG)/9.81,800),smooth(abs(RAMP_STEER_R.front_slip_angle)-abs(RAMP_STEER_R.rear_slip_angle),800),'DisplayName','Handling Diagram')
xlabel('ayG/g');ylabel('slip difference');
title('RAMP_STEER_R');
legend
subplot(2,2,4)
scatter(smooth(abs(RAMP_STEER_L.ayG)/9.81,800),smooth(abs(RAMP_STEER_L.front_slip_angle-RAMP_STEER_L.rear_slip_angle),800),'DisplayName','Handling Diagram')
xlabel('ayG/g');ylabel('slip difference');
title('RAMP_STEER_L');
legend
%% Understeering gradient
Kus_1 = mean(smooth(abs(SP_100FT_CR_IS_CW.front_slip_angle)-abs(SP_100FT_CR_IS_CW.rear_slip_angle),800)/vehicle.L./smooth(abs(SP_100FT_CR_IS_CW.ayG),800));
Kus_2 = mean(smooth(abs(SP_100FT_CR_IS_CCW.front_slip_angle)-abs(SP_100FT_CR_IS_CCW.rear_slip_angle),800)/vehicle.L./smooth(abs(SP_100FT_CR_IS_CCW.ayG),800));
Kus_3 = mean(smooth(abs(RAMP_STEER_L.front_slip_angle)-abs(RAMP_STEER_L.rear_slip_angle),800)/vehicle.L./smooth(abs(RAMP_STEER_L.ayG),800));
Kus_4 = mean(smooth(abs(RAMP_STEER_R.front_slip_angle)-abs(RAMP_STEER_R.rear_slip_angle),800)/vehicle.L./smooth(abs(RAMP_STEER_R.ayG),800));


Kus = mean([Kus_1,Kus_2,Kus_3,Kus_4]);









