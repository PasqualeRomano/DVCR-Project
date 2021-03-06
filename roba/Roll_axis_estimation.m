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

load("Dataset/STRAIGHT_LINE_0.mat");
load("Dataset/STRAIGHT_LINE_1.mat");
load("Dataset/RAMP_STEER_L.mat");
load("Dataset/RAMP_STEER_R.mat");
load("Dataset/SP_100FT_CR_IS_CCW.mat");
load("Dataset/SP_100FT_CR_IS_CW.mat");
load("roll_rad.mat","R_dyn")

%% Static front load
Fz_F = vehicle.mass*vehicle.Lr/vehicle.L*9.81;
%Fzfr - Fzf/2 = DY
%load transfer ramp steer
DeltaZF_RSL =  (RAMP_STEER_L.Fz_FR - Fz_F/2 - RAMP_STEER_L.Fz_FL + Fz_F/2)./2;
DeltaZF_RSR =  (RAMP_STEER_R.Fz_FR - Fz_F/2 - RAMP_STEER_R.Fz_FL + Fz_F/2)./2;
DeltaZF_CCW =  (SP_100FT_CR_IS_CCW.Fz_FR - Fz_F/2 - SP_100FT_CR_IS_CCW.Fz_FL + Fz_F/2)./2;
DeltaZF_CW =  (SP_100FT_CR_IS_CW.Fz_FR - Fz_F/2 - SP_100FT_CR_IS_CW.Fz_FL + Fz_F/2)./2;
%%
%Check steering angles left-right
figure
yyaxis left 
subplot(2,1,1)
plot(RAMP_STEER_L.time,smooth(DeltaZF_RSL,800),'DisplayName','DeltaZF_RSL')
xlabel('time (s)');ylabel('N');
legend
hold on
yyaxis right
plot(RAMP_STEER_L.time,smooth(RAMP_STEER_L.ayG,800),'DisplayName','ayG')
xlabel('time (s)');ylabel('m/s^2');
title('RAMP_STEER_L');
legend

subplot(2,1,2)
yyaxis left
plot(RAMP_STEER_R.time,smooth(DeltaZF_RSR,800),'DisplayName','DeltaZF_RSR')
xlabel('time (s)');ylabel('N');
legend
hold on
yyaxis right
plot(RAMP_STEER_R.time,smooth(RAMP_STEER_R.ayG,800),'DisplayName','ayG')
xlabel('time (s)');ylabel('m/s^2');
title('RAMP_STEER_R');
legend

figure
subplot(2,1,1)
yyaxis left
plot(SP_100FT_CR_IS_CCW.time,smooth(DeltaZF_CCW,800),'DisplayName','DeltaZF_CCW')
xlabel('time (s)');ylabel('N');
legend
hold on
yyaxis right
plot(SP_100FT_CR_IS_CCW.time,smooth(SP_100FT_CR_IS_CCW.ayG,800),'DisplayName','ayG')
xlabel('time (s)');ylabel('m,/s^2');
title('SP_100FT_CR_IS_CCW');
legend

subplot(2,1,2)
yyaxis left
plot(SP_100FT_CR_IS_CW.time,smooth(DeltaZF_CW,800),'DisplayName','DeltaZF_CW')
legend
hold on
yyaxis right
plot(SP_100FT_CR_IS_CW.time,smooth(SP_100FT_CR_IS_CW.ayG,800),'DisplayName','ayG')
xlabel('time (s)');ylabel('m,/s^2');
title('SP_100FT_CR_IS_CW');
legend
%%
%DFzf = mayG/wr(Lr/L*hr+hgs K/2*K)

ft = fittype( @(hr,x) vehicle.mass*x/vehicle.W.*(vehicle.Lr/vehicle.L*hr+vehicle.h_Gs/2));

fit_kroll_RSL = fit(smooth(RAMP_STEER_L.ayG,500),smooth(DeltaZF_RSL,500),ft,'StartPoint',0);
h_roll_RSL = fit_kroll_RSL.hr

fit_kroll_RSR = fit(smooth(RAMP_STEER_R.ayG,500),smooth(DeltaZF_RSR,500),ft,'StartPoint',0);
h_roll_RSR = fit_kroll_RSR.hr

fit_kroll_CCW = fit(smooth(SP_100FT_CR_IS_CCW.ayG,500),smooth(DeltaZF_CCW,500),ft,'StartPoint',0);
h_roll_CCW = fit_kroll_CCW.hr

fit_kroll_CW = fit(smooth(SP_100FT_CR_IS_CW.ayG,500),smooth(DeltaZF_CW,500),ft,'StartPoint',0);
h_roll_CW = fit_kroll_CW.hr

h_roll= (h_roll_RSL+h_roll_RSR+h_roll_CCW+h_roll_CW)/4;
figure
plot(RAMP_STEER_L.time,vehicle.mass*smooth(RAMP_STEER_L.ayG,500)/vehicle.W.*(vehicle.Lr/vehicle.L*h_roll_RSL+vehicle.h_Gs/2),'DisplayName','DeltaZF_RSL fitted')
xlabel('time (s)');ylabel('N');
legend
hold on
plot(RAMP_STEER_L.time,smooth(DeltaZF_RSL,500),'DisplayName','DeltaZF_RSL')
xlabel('time (s)');ylabel('N');
title('RAMP_STEER_L');
legend

save('roll_height','h_roll');