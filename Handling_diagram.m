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


Fy_R_CCW = vehicle.mass* SP_100FT_CR_IS_CCW.ayG *vehicle.Lf/vehicle.L;
Fy_R_CW = vehicle.mass* SP_100FT_CR_IS_CW.ayG *vehicle.Lf/vehicle.L;
Fy_R_RSL = vehicle.mass* RAMP_STEER_L.ayG *vehicle.Lf/vehicle.L;
Fy_R_RSR= vehicle.mass* RAMP_STEER_R.ayG *vehicle.Lf/vehicle.L;


%Check axles charateristics
figure
subplot(2,2,1)
scatter(abs(smooth(SP_100FT_CR_IS_CCW.front_slip_angle,800)),abs(smooth(SP_100FT_CR_IS_CCW.Fy_FL+SP_100FT_CR_IS_CCW.Fy_FR,800)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(SP_100FT_CR_IS_CCW.rear_slip_angle,800)),abs(smooth(Fy_R_CCW,800)),'DisplayName','rear axle charateristic')
xlabel('slip');ylabel('Fy');
title('SP_100FT_CR_IS_CCW');
legend

subplot(2,2,2)
scatter(abs(smooth(SP_100FT_CR_IS_CW.front_slip_angle,800)),abs(smooth(SP_100FT_CR_IS_CW.Fy_FL+SP_100FT_CR_IS_CW.Fy_FR,800)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(SP_100FT_CR_IS_CW.rear_slip_angle,800)),abs(smooth(Fy_R_CW,800)),'DisplayName','rear axle charateristic')
xlabel('slip');ylabel('Fy');
title('SP_100FT_CR_IS_CW');
legend

subplot(2,2,3)
scatter(abs(smooth(RAMP_STEER_L.front_slip_angle,800)),abs(smooth(RAMP_STEER_L.Fy_FL+RAMP_STEER_L.Fy_FR,800)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(RAMP_STEER_L.rear_slip_angle,800)),abs(smooth(Fy_R_RSL,800)),'DisplayName','rear axle charateristic')
xlabel('slip');ylabel('Fy');
title('RAMP_STEER_L');
legend

subplot(2,2,4)
scatter(abs(smooth(RAMP_STEER_R.front_slip_angle,800)),abs(smooth(RAMP_STEER_R.Fy_FL+RAMP_STEER_R.Fy_FR,800)),'DisplayName','front axle charateristic')
hold on
scatter(abs(smooth(RAMP_STEER_R.rear_slip_angle,800)),abs(smooth(Fy_R_RSR,800)),'DisplayName','rear axle charateristic')
xlabel('slip');ylabel('Fy');
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
 
 
 vKus = mean([Kus_1,Kus_2,Kus_3,Kus_4]);

% TRY POLYFIT AND EXTRACT KUS AS DERIVATIV IN 0)


%% Steering gradients--> rho
load("steer_ratio.mat","tau_VW_mean");

%rho estimate
%rho = (dh*tau + alphar-alphaf)1/L
rho_RSL = RAMP_STEER_L.delta_HW/tau_VW_mean/vehicle.L + (RAMP_STEER_L.rear_slip_angle-RAMP_STEER_L.front_slip_angle)/vehicle.L;
rho_RSR = RAMP_STEER_R.delta_HW/tau_VW_mean/vehicle.L + (RAMP_STEER_R.rear_slip_angle-RAMP_STEER_R.front_slip_angle)/vehicle.L;
figure
scatter(smooth(RAMP_STEER_L.ayG,800),smooth(rho_RSL,800),'DisplayName','rho(ay) RSL')
xlabel('ayG');ylabel('rho');
hold on 
scatter(smooth(RAMP_STEER_R.ayG,800),smooth(rho_RSR,800),'DisplayName','rho(ay) RSR')
xlabel('ayG');ylabel('rho');
legend

rho_ay_RSL = fit(RAMP_STEER_L.ayG,rho_RSL,'poly1','Exclude', RAMP_STEER_L.ayG>7);
rho_ay_RSR = fit(RAMP_STEER_R.ayG,rho_RSR,'poly1','Exclude', RAMP_STEER_R.ayG<-7);


figure
plot(RAMP_STEER_L.ayG,rho_RSL,'DisplayName','rho(ay)')
hold on
plot(RAMP_STEER_L.ayG,rho_ay_RSL(RAMP_STEER_L.ayG),'DisplayName','rho(ay) fitted')
xlabel('ayG');ylabel('rho');
title('RAMP_STEER_L');
legend
figure
plot(RAMP_STEER_R.ayG,rho_RSR,'DisplayName','rho(ay)')
hold on
plot(RAMP_STEER_R.ayG,rho_ay_RSR(RAMP_STEER_R.ayG),'DisplayName','rho(ay) fitted')
xlabel('ayG');ylabel('rho');
title('RAMP_STEER_R');
legend
%steering gradient of rho wrt ay
K_rho_ay_RSL = rho_ay_RSL.p1
K_rho_ay_RSR = rho_ay_RSR.p1


rho_CW = (SP_100FT_CR_IS_CW.delta_HW/tau_VW_mean)/vehicle.L + (SP_100FT_CR_IS_CW.rear_slip_angle-SP_100FT_CR_IS_CW.front_slip_angle)/vehicle.L;
rho_CCW = (SP_100FT_CR_IS_CCW.delta_HW/tau_VW_mean)/vehicle.L + (SP_100FT_CR_IS_CCW.rear_slip_angle-SP_100FT_CR_IS_CCW.front_slip_angle)/vehicle.L;

figure
scatter(smooth(SP_100FT_CR_IS_CW.delta_HW,800),smooth(rho_CW,800),'DisplayName','rho(delta) CW')
xlabel('delta');ylabel('rho');
hold on 
scatter(smooth(SP_100FT_CR_IS_CCW.delta_HW,800),smooth(rho_CCW,800),'DisplayName','rho(delta) CCW')
xlabel('delta');ylabel('rho');
legend

rho_delta_CW = fit(SP_100FT_CR_IS_CW.delta_HW,rho_CW,'poly1');
rho_delta_CCW = fit(SP_100FT_CR_IS_CCW.delta_HW,rho_CCW,'poly1');

figure
plot(SP_100FT_CR_IS_CW.delta_HW,rho_CW,'DisplayName','rho(ay)')
hold on
plot(SP_100FT_CR_IS_CW.delta_HW,rho_delta_CW(SP_100FT_CR_IS_CW.delta_HW),'DisplayName','rho(delta) fitted')
xlabel('delta');ylabel('rho');
title('SP_100FT_CR_IS_CW');
legend
figure
plot(SP_100FT_CR_IS_CCW.delta_HW,rho_CCW,'DisplayName','rho(ay)')
hold on
plot(SP_100FT_CR_IS_CCW.delta_HW,rho_delta_CCW(SP_100FT_CR_IS_CCW.delta_HW),'DisplayName','rho(delta) fitted')
xlabel('delta');ylabel('rho');
title('SP_100FT_CR_IS_CCW');
legend
%steering of rho gradint wrt delta
K_rho_delta_CW = rho_delta_CW.p1
K_rho_delta_CW = rho_delta_CCW.p1
%% Steering gradients--> beta
beta_RSL = vehicle.Lr/vehicle.L*RAMP_STEER_L.delta_HW*1/tau_VW_mean-(RAMP_STEER_L.rear_slip_angle*vehicle.Lf+RAMP_STEER_L.front_slip_angle*vehicle.Lr)/vehicle.L;
beta_RSR = vehicle.Lr/vehicle.L*RAMP_STEER_R.delta_HW*1/tau_VW_mean-(RAMP_STEER_R.rear_slip_angle*vehicle.Lf+RAMP_STEER_R.front_slip_angle*vehicle.Lr)/vehicle.L;
figure
scatter(smooth(RAMP_STEER_L.ayG,800),smooth(beta_RSL,800),'DisplayName','beta(ay) RSL')
xlabel('ayG');ylabel('beta');
hold on
scatter(smooth(RAMP_STEER_R.ayG,800),smooth(beta_RSR,800),'DisplayName','beta(ay) RSR')
xlabel('ayG');ylabel('beta');


beta_ay_RSL = fit(RAMP_STEER_L.ayG,beta_RSL,'poly1','Exclude', RAMP_STEER_L.ayG>6);
beta_ay_RSR = fit(RAMP_STEER_R.ayG,beta_RSR,'poly1','Exclude', RAMP_STEER_R.ayG<-6);

figure
plot(RAMP_STEER_L.ayG,beta_RSL,'DisplayName','beta(ay)')
hold on
plot(RAMP_STEER_L.ayG,beta_ay_RSL(RAMP_STEER_L.ayG),'DisplayName','beta(ay) fitted')
xlabel('ayg');ylabel('beta');
title('RAMP_STEER_L');
legend
figure
plot(RAMP_STEER_R.ayG,beta_RSR,'DisplayName','beta(ay)')
hold on
plot(RAMP_STEER_R.ayG,beta_ay_RSR(RAMP_STEER_R.ayG),'DisplayName','beta(ay) fitted')
xlabel('ayg');ylabel('beta');
title('RAMP_STEER_R');
legend
%steering of beta gradient wrt ay
K_beta_ay_RSL = beta_ay_RSL.p1
K_beta_ay_RSR = beta_ay_RSR.p1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
beta_CCW = vehicle.Lr/vehicle.L*SP_100FT_CR_IS_CCW.delta_HW*1/tau_VW_mean-(SP_100FT_CR_IS_CCW.rear_slip_angle*vehicle.Lf+SP_100FT_CR_IS_CCW.front_slip_angle*vehicle.Lr)/vehicle.L;
beta_CW = vehicle.Lr/vehicle.L*SP_100FT_CR_IS_CW.delta_HW*1/tau_VW_mean-(SP_100FT_CR_IS_CW.rear_slip_angle*vehicle.Lf+SP_100FT_CR_IS_CW.front_slip_angle*vehicle.Lr)/vehicle.L;
figure
scatter(smooth(SP_100FT_CR_IS_CCW.delta_HW,800),smooth(beta_CCW,800),'DisplayName','beta(delta) CCW')
xlabel('delta');ylabel('beta');
hold on
scatter(smooth(SP_100FT_CR_IS_CW.delta_HW,800),smooth(beta_CW,800),'DisplayName','beta(delta) CW')
xlabel('delta');ylabel('beta');


beta_delta_CCW = fit(SP_100FT_CR_IS_CCW.delta_HW,beta_CCW,'poly1','Exclude', SP_100FT_CR_IS_CCW.delta_HW>6);
beta_delta_CW = fit(SP_100FT_CR_IS_CW.delta_HW,beta_CW,'poly1','Exclude', SP_100FT_CR_IS_CW.delta_HW<-6);

figure
plot(SP_100FT_CR_IS_CCW.delta_HW,beta_CCW,'DisplayName','beta(ay)')
hold on
plot(SP_100FT_CR_IS_CCW.delta_HW,beta_delta_CCW(SP_100FT_CR_IS_CCW.delta_HW),'DisplayName','beta(ay) fitted')
xlabel('delta');ylabel('beta');
title('SP_100FT_CR_IS_CCW');
legend
figure
plot(SP_100FT_CR_IS_CW.delta_HW,beta_CW,'DisplayName','beta(ay)')
hold on
plot(SP_100FT_CR_IS_CW.delta_HW,beta_delta_CW(SP_100FT_CR_IS_CW.delta_HW),'DisplayName','beta(ay) fitted')
xlabel('delta');ylabel('beta');
title('SP_100FT_CR_IS_CW');
legend

%steering of beta gradient wrt delta
K_beta_delta_CCW = beta_delta_CCW.p1
K_beta_delta_CW = beta_delta_CW.p1