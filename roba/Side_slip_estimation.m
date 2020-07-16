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


%tan alha = Vcy/|Vcx|
%side_slip_arctan(SP_100FT_CR_IS_CCW.latvel./abs(P_100FT_CR_IS_CCW.long_vel))
figure
plot(SP_100FT_CR_IS_CCW.time,SP_100FT_CR_IS_CCW.ayG/9.81,'DisplayName','ayG CCW')
hold on
plot(SP_100FT_CR_IS_CW.time,SP_100FT_CR_IS_CW.ayG/9.81,'DisplayName','ayG CW')
xlabel('time (s)');ylabel('m/s^2');

side_slip_front_CCW = (SP_100FT_CR_IS_CCW.Fy_FL(20/0.2:60/0.2)+SP_100FT_CR_IS_CCW.Fy_FR(20/0.2:60/0.2))/vehicle.C_a_f;

side_slip_rear_CCW = vehicle.mass*SP_100FT_CR_IS_CCW.ayG(20/0.2:60/0.2)*vehicle.Lf/vehicle.L/vehicle.C_a_r;
side_slip_front_CW = (SP_100FT_CR_IS_CW.Fy_FL(20/0.2:60/0.2)+SP_100FT_CR_IS_CW.Fy_FR(20/0.2:60/0.2))/vehicle.C_a_f;

side_slip_rear_CW = vehicle.mass*abs(SP_100FT_CR_IS_CW.ayG(20/0.2:60/0.2))*vehicle.Lf/vehicle.L/vehicle.C_a_r;

figure
subplot(2,1,1)
plot(SP_100FT_CR_IS_CCW.time(20/0.2:60/0.2),side_slip_front_CCW,'DisplayName','estimated side front side slip')
hold on
plot(SP_100FT_CR_IS_CCW.time(20/0.2:60/0.2),SP_100FT_CR_IS_CCW.front_slip_angle(20/0.2:60/0.2),'DisplayName','given front side slip')
xlabel('time (s)');ylabel('slip');
title('SP_100FT_CR_IS_CCW - front');
subplot(2,1,2)
plot(SP_100FT_CR_IS_CCW.time(20/0.2:60/0.2),side_slip_rear_CCW,'DisplayName','estimated side rear side slip')
hold on
plot(SP_100FT_CR_IS_CCW.time(20/0.2:60/0.2),SP_100FT_CR_IS_CCW.rear_slip_angle(20/0.2:60/0.2),'DisplayName','given rear side slip')
xlabel('time (s)');ylabel('slip');
title('SP_100FT_CR_IS_CCW - rear');

figure
subplot(2,1,1)
plot(SP_100FT_CR_IS_CW.time(20/0.2:60/0.2),side_slip_front_CW,'DisplayName','estimated side front side slip')
hold on
plot(SP_100FT_CR_IS_CW.time(20/0.2:60/0.2),SP_100FT_CR_IS_CW.front_slip_angle(20/0.2:60/0.2),'DisplayName','given front side slip')
xlabel('time (s)');ylabel('slip');
title('SP_100FT_CR_IS_CW - front');
subplot(2,1,2)
plot(SP_100FT_CR_IS_CW.time(20/0.2:60/0.2),side_slip_rear_CW,'DisplayName','estimated side rear side slip')
hold on
plot(SP_100FT_CR_IS_CW.time(20/0.2:60/0.2),SP_100FT_CR_IS_CW.rear_slip_angle(20/0.2:60/0.2),'DisplayName','given rear side slip')
xlabel('time (s)');ylabel('slip');
title('SP_100FT_CR_IS_CW - rear');

%% tyre slips

u_CCW = sqrt(SP_100FT_CR_IS_CCW.lat_vel.^2+SP_100FT_CR_IS_CCW.long_vel.^2).*cos(SP_100FT_CR_IS_CCW.body_slip);
v_CCW = sqrt(SP_100FT_CR_IS_CCW.lat_vel.^2+SP_100FT_CR_IS_CCW.long_vel.^2).*sin(SP_100FT_CR_IS_CCW.body_slip); % we don't give a shit of small angles
side_slip_fl_CCW = -SP_100FT_CR_IS_CCW.delta_L+(v_CCW+SP_100FT_CR_IS_CCW.yaw_rate*vehicle.Lf)./u_CCW;
side_slip_fr_CCW = -SP_100FT_CR_IS_CCW.delta_R+(v_CCW+SP_100FT_CR_IS_CCW.yaw_rate*vehicle.Lf)./u_CCW;

side_slip_rl_CCW = (v_CCW-SP_100FT_CR_IS_CCW.yaw_rate*vehicle.Lf)./u_CCW;
side_slip_rr_CCW = (v_CCW-SP_100FT_CR_IS_CCW.yaw_rate*vehicle.Lf)./u_CCW;

figure
subplot(2,1,1)
plot(SP_100FT_CR_IS_CCW.time,SP_100FT_CR_IS_CCW.front_slip_angle,'DisplayName','front axle side slip')
hold on
plot(SP_100FT_CR_IS_CCW.time,side_slip_fl_CCW,'DisplayName','front left side slip')
hold on
plot(SP_100FT_CR_IS_CCW.time,side_slip_fr_CCW,'DisplayName','front right side slip')
xlabel('time (s)');ylabel('slip');
title('SP_100FT_CR_IS_CCW - front');
subplot(2,1,2)
plot(SP_100FT_CR_IS_CCW.time,SP_100FT_CR_IS_CCW.rear_slip_angle,'DisplayName','rear axle side slip')
hold on
plot(SP_100FT_CR_IS_CCW.time,side_slip_rl_CCW,'DisplayName','rear left side slip')
hold on
plot(SP_100FT_CR_IS_CCW.time,side_slip_rr_CCW,'DisplayName','rear right side slip')
xlabel('time (s)');ylabel('slip');
title('SP_100FT_CR_IS_CCW - rear');