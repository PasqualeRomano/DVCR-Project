%% MODELLING AND SIMULATION OF A SINGLE TRACK MODEL
close all
clear all
clc
load("roll_rad.mat","R_dyn")
load("steer_ratio.mat","tau_VW_mean")
load("CoM.mat","hG")
load('SP_100FT_CR_IS_CCW.mat')
load('SP_100FT_CR_IS_CW.mat')

p1_parameters
%% Fit B,C,D,E magic formula
% kappa_opt_F = SP_100FT_CR_IS_CCW.front_slip_angle;
% errF = @(x) rms((-SP_100FT_CR_IS_CCW.Fy_FL-SP_100FT_CR_IS_CCW.Fy_FR)- (x(3)*sin(x(2)*atan(kappa_opt_F*x(1)-(kappa_opt_F*x(1)-atan(kappa_opt_F*x(1))*x(4))))));
% x0 = [7 2 0.95 0.9];
% options.MaxFunEvals = 3000;
% options.MaxIter = 2000;
% x_opt_F = fminsearch(errF,x0,options);
% figure
% scatter(SP_100FT_CR_IS_CCW.front_slip_angle,(-SP_100FT_CR_IS_CCW.Fy_FL-SP_100FT_CR_IS_CCW.Fy_FR))
% hold on
% plot(SP_100FT_CR_IS_CCW.front_slip_angle,(x_opt_F(3)*sin(x_opt_F(2)*atan(kappa_opt_F*x_opt_F(1)-(kappa_opt_F*x_opt_F(1)-atan(kappa_opt_F*x_opt_F(1))*x_opt_F(4))))))
% title("Fit front")
% x_opt_F = abs(x_opt_F);
% 
% kappa_opt_R = SP_100FT_CR_IS_CCW.rear_slip_angle;
% errR = @(x) rms(-vehicle.mass*SP_100FT_CR_IS_CCW.ayG*vehicle.Lf/vehicle.L- (x(3)*sin(x(2)*atan(kappa_opt_R*x(1)-(kappa_opt_R*x(1)-atan(kappa_opt_R*x(1))*x(4))))));
% x0 = [5 2 1 0.9];
% options.MaxFunEvals = 3000;
% options.MaxIter = 2000;
% x_opt_R = fminsearch(errR,x0,options);
% figure
% scatter(SP_100FT_CR_IS_CCW.rear_slip_angle,-vehicle.mass*SP_100FT_CR_IS_CCW.ayG*vehicle.Lf/vehicle.L)
% hold on
% plot(SP_100FT_CR_IS_CCW.rear_slip_angle,(x_opt_R(3)*sin(x_opt_R(2)*atan(kappa_opt_R*x_opt_R(1)-(kappa_opt_R*x_opt_R(1)-atan(kappa_opt_R*x_opt_R(1))*x_opt_R(4))))))
% title("Fit rear")
%x_opt_R = abs(x_opt_R);

Fy_F = SP_100FT_CR_IS_CCW.Fy_FL+SP_100FT_CR_IS_CCW.Fy_FR;
Fz_F = SP_100FT_CR_IS_CCW.Fz_FL+SP_100FT_CR_IS_CCW.Fz_FR;
ft = fittype(@(B,C,D,E,x) D*sin(C*atan(x*B-(x*B-atan(x*B))*E)));
fit_magic_front = fit(SP_100FT_CR_IS_CCW.front_slip_angle,-Fy_F./Fz_F,ft,'StartPoint',[7 2 .95 .9]);

figure
scatter(SP_100FT_CR_IS_CCW.front_slip_angle,-Fy_F)
 hold on
 plot(SP_100FT_CR_IS_CCW.front_slip_angle,fit_magic_front(SP_100FT_CR_IS_CCW.front_slip_angle).*Fz_F)
 title("Fit front")


Fy_R = vehicle.mass*SP_100FT_CR_IS_CCW.ayG*vehicle.Lf/vehicle.L;
Fz_R = vehicle.mass*9.81-(SP_100FT_CR_IS_CCW.Fz_FL+SP_100FT_CR_IS_CCW.Fz_FR);
fit_magic_rear = fit(SP_100FT_CR_IS_CCW.rear_slip_angle,-Fy_R./Fz_R,ft,'StartPoint',[5 2 1 0.9]);

figure
scatter(SP_100FT_CR_IS_CCW.rear_slip_angle,-Fy_R)
 hold on
 plot(SP_100FT_CR_IS_CCW.rear_slip_angle,fit_magic_rear(SP_100FT_CR_IS_CCW.rear_slip_angle).*Fz_R)
 title("Fit rear")




%%
% All the system parameters are stred in a vector in order to easely pass the 
% parameters to the Simulink model.
% Model parameters
data.gravity = 9.81;
data.mass    =  vehicle.mass;   
data.Iz      = vehicle.Izz;
data.Lr      = vehicle.Lr;
data.Lf      =vehicle.Lf;
data.kv      = 0.0;
data.h       = hG;

% tyre parameters  %     
data.Byf     = fit_magic_front.B; %7.0
data.Cyf     = fit_magic_front.C;
data.Dyf     = fit_magic_front.D;
data.Eyf     = fit_magic_front.E;
data.Kyf     = data.Byf*data.Cyf*data.Dyf;

data.Byr     = fit_magic_rear.B;
data.Cyr     = fit_magic_rear.C;
data.Dyr     = fit_magic_rear.D;
data.Eyr     = fit_magic_rear.E;
data.Kyr     = data.Byr*data.Cyr*data.Dyr;

data.tyre_model = 1; %'linear';
data.tyre_model = 2; %'non_linear';


figure()
alpha = [-0.5:0.01:0.5];
FYR    = magic_formula(alpha,data.Byr,data.Cyr,data.Dyr,data.Eyr);
FYF    = magic_formula(alpha,data.Byf,data.Cyf,data.Dyf,data.Eyf);
plot(alpha,FYR); hold on
plot(alpha,FYF,'--r','LineWidth',2)
legend('rear','front')
%% INPUT
mane_type = 1;
%maneouvre type:

%1 -> STEP STEER
if mane_type==1

load('STEP_STEER.mat')
%delta
Ts = STEP_STEER.time(2)-STEP_STEER.time(1);
in_delta = timeseries(STEP_STEER.delta_HW./tau_VW_mean,0:Ts:(length(STEP_STEER.delta_HW)-1)*Ts);
%vel
ref_vel = timeseries(STEP_STEER.long_vel,0:Ts:(length(STEP_STEER.delta_HW)-1)*Ts);
%traction
U0 = mean(STEP_STEER.long_vel);
kp     = 500;      % Gain P controller for velocity tracking
figure()
plot(in_delta)
title('Steering input')
ylabel('delta (deg)')
xlabel('time (s)')
end


if mane_type == 0
% % Parameters used in the step blocks
 U0     = 95.0/3.6; % Desired forward velocity
 kp     = 500;      % Gain P controller for velocity tracking
 delta0 = 0.01;     % steer step
 t0     = 5;        % time step begin step
end
fig_name = ['./figures/fig_dyn_characteristics_kv',num2str(data.kv),'.pdf'];
%% Model without relaxation length
% Load model without tyre dynamics and run the simulation
mdl1 = 'single_track';
load_system(mdl1)
% configSetNames = get_param(cs, 'ObjectParameters') 

% Prepare model configuration 
conf1 = getActiveConfigSet(mdl1);
cs1 = conf1.copy();
% Set up simulation parameters
set_param(cs1,'MaxStep'           , '0.1');
set_param(cs1,'AbsTol'            , '1e-6');
set_param(cs1,'StartTime'         , '0');
set_param(cs1, 'StopTime', num2str(in_delta.time(end)));
set_param(cs1,'SaveTime'          , 'on');
set_param(cs1,'TimeSaveName'      , 'tout');
set_param(cs1,'SaveState'         , 'on');
set_param(cs1,'StateSaveName'     , 'xout');
set_param(cs1,'SaveOutput'        , 'on');
set_param(cs1,'OutputSaveName'    , 'yout');
set_param(cs1,'LimitDataPoints'   , 'off');
set_param(cs1,'SignalLogging'     , 'on');
set_param(cs1,'SignalLoggingName' , 'logsout');
set_param(cs1,'Solver'            , 'ode23tb'); %'ode15s'; % ode23s

% Set the state initial condition using a structure (to avoid warning)
xInitial1 = [STEP_STEER.x_pos(1), STEP_STEER.y_pos(1), 0,  ... %positions
             STEP_STEER.long_vel(1),STEP_STEER.lat_vel(1) , 0]; % velocities
init_states1.time = 0;
init_states1.signals.values  =  xInitial1; 
init_states1.signals.label   = 'CSTATE';
init_states1.signals.blockName  ='single_track/Integrator';
set_param(cs1,'LoadInitialState' ,'on');
set_param(cs1,'InitialState','init_states1');

%Simulate the model
fprintf(1,'Simulating model \n tyre stiffnesses [Kyr, Kyf] = [%6.3f , %6.3f]\n', data.Kyr, data.Kyf)
simOut1 = sim(mdl1,cs1);

%% Plots: compare results
time1   = simOut1.get('tout');
res1    = simOut1.get('xout');
logsout = simOut1.get('logsout');
outputs = simOut1.get('outputs');
input   = logsout.getElement('input');
Fxr     = input.Values.Data(:,1);
Fxf     = input.Values.Data(:,2);
delta   = input.Values.Data(:,3);
x       = res1(:,1);
y       = res1(:,2);
psi     = res1(:,3);
u       = res1(:,4);
v       = res1(:,5);
Omega   = res1(:,6);
Fzr     = outputs(:,1); 
Fzf     = outputs(:,2);
alpha_r = outputs(:,3); 
alpha_f = outputs(:,4);
Fyr     = outputs(:,5); 
Fyf     = outputs(:,6);


figure('Position',[0,0,800,800])
subplot(6,1,1)
plot(time1,u*3.6,'LineWidth',2); hold on
plot(time1,v*3.6,'-r','LineWidth',2)
title('Velocity')
legend('u','v')
%ylim([-20 100])

subplot(6,1,2)
plot(time1,Omega*180/pi,'LineWidth',2); hold on
title('Yaw rate')
ylabel('(deg)')

subplot(6,1,3)
plot(x,y,'LineWidth',2); hold on
title('trajectory')
axis equal

subplot(6,1,4)
plot(time1,Omega.*u/9.81,'LineWidth',2); hold on
title('lateral acceleration')
%ylim([-1 1])

subplot(6,1,5)
plot(time1,Fxr,'LineWidth',2); hold on
plot(time1,Fxf,'-r','LineWidth',2);
legend('F_{x_r}','F_{x_f}')
title('long forces')

subplot(6,1,6)
plot(time1,delta*180/pi,'LineWidth',2); hold on
ylabel('(deg)')
title('steer angle')

%
figure()
subplot(3,1,1)
plot(time1,alpha_r); hold on
plot(time1,alpha_f,'-r')
title('side slip angles')
legend('alpha_r','alpha_f')

subplot(3,1,2)
plot(time1,Fyr); hold on
plot(time1,Fyf,'-r')
title('side slip angles')
legend('F_{y_r}','F_{y_f}')

subplot(3,1,3)
plot(alpha_r,Fyr./Fzr); hold on
plot(alpha_f,Fyf./Fzf,'-r')
title('side slip angles')
legend('F_{y_r}/Nr','F_{y_f}/Nf')

% %% Parametric analysis
% %  Constant steering axis and variable forward velocity
% U0vec     = [10:5:100]/3.6;
% figure('Position',[0,0,800,800])
% 
% %neutral
% data.Kyr = 7.8;
% data.Kyf = 7.8;
% ssa = [0:0.01:0.1];
% Fr_N = magic_formula(ssa,data.Byr,data.Cyr,data.Dyr,data.Eyr);
% Ff_N = magic_formula(ssa,data.Byf,data.Cyf,data.Dyf,data.Eyf);
% 
% parametric_simulations
% res_par_N = res_par;
% 
% % Understeering
% data.Kyr  = 10.0;
% data.Byr  = data.Kyr/(data.Dyr*data.Cyr);
% data.Kyf  =  7.8;
% data.Byf  = data.Kyf/(data.Dyf*data.Cyf);
% Fr_U = magic_formula(ssa,data.Byr,data.Cyr,data.Dyr,data.Eyr);
% Ff_U = magic_formula(ssa,data.Byf,data.Cyf,data.Dyf,data.Eyf);
% parametric_simulations
% res_par_U = res_par;
% 
% % Oversteering
% data.Kyr  = 7.8;
% data.Byr  = data.Kyr/(data.Dyr*data.Cyr);
% data.Kyf  = 10.0;
% data.Byf  = data.Kyf/(data.Dyf*data.Cyf);
% Fr_O = magic_formula(ssa,data.Byr,data.Cyr,data.Dyr,data.Eyr);
% Ff_O = magic_formula(ssa,data.Byf,data.Cyf,data.Dyf,data.Eyf);
% parametric_simulations
% res_par_O = res_par;
% 
% 
% %%
% figure('Position',[0,0,600,800])
% subplot(5,3,[1,2,3])
% plot(U0vec,res_par_N.Rvec,'-xg'); hold on
% plot(U0vec,res_par_O.Rvec,'-xr')
% plot(U0vec,res_par_U.Rvec,'-xb')
% legend('N','O','U')
% title(['Curvature radius with constant steering angle-kv =',num2str(data.kv)])
% ylabel('curvature radius (m)')
% xlabel('forward velocity (m/s)')
% set(gca,'YGrid','on')
% 
% subplot(5,3,[4,5,6])
% plot(res_par_N.OmegaVec.*U0vec,res_par_N.Rvec,'-xg'); hold on
% plot(res_par_O.OmegaVec.*U0vec,res_par_O.Rvec,'-xr')
% plot(res_par_U.OmegaVec.*U0vec,res_par_U.Rvec,'-xb')
% plot(res_par_N.OmegaVec.*U0vec,res_par_N.RvecTrue,'--g')
% plot(res_par_O.OmegaVec.*U0vec,res_par_O.RvecTrue,'--r')
% plot(res_par_U.OmegaVec.*U0vec,res_par_U.RvecTrue,'--b')
% legend('N','O','U')
% ylabel('curvature radius (m)')
% xlabel('Lateral acceleration (m/s^2)')
% set(gca,'YGrid','on')
% 
% subplot(5,3,[7,8,9])
% plot(U0vec,res_par_N.FXFvec,'-xg'); hold on
% plot(U0vec,res_par_O.FXFvec,'-xr')
% plot(U0vec,res_par_U.FXFvec,'-xb')
% legend('N','O','U')
% ylabel('Longitudinal force (N)')
% xlabel('forward velocity (m/s)')
% 
% subplot(5,3,[10,11,12])
% plot(U0vec,res_par_N.BetaVec*180/pi,'-xg'); hold on
% plot(U0vec,res_par_O.BetaVec*180/pi,'-xr')
% plot(U0vec,res_par_U.BetaVec*180/pi,'-xb')
% legend('N','O','U')
% ylabel('\beta (deg)')
% xlabel('forward velocity (m/s)')
% set(gca,'YGrid','on')
% 
% subplot(5,3,13)
% plot(res_par_N.alphaRVec,res_par_N.fyRVec,'xb'); hold on
% plot(ssa,Fr_N,'--b');
% plot(res_par_N.alphaFVec,res_par_N.fyFVec,'or'); 
% plot(ssa,Ff_N,'--r');
% title('Neutral')
% xlabel('side slip (rad)')
% ylabel('Norm. lat. accel. ()')
% 
% subplot(5,3,14)
% plot(res_par_O.alphaRVec,res_par_O.fyRVec,'xb'); hold on
% plot(ssa,Fr_O,'--b');
% plot(res_par_O.alphaFVec,res_par_O.fyFVec,'or'); 
% plot(ssa,Ff_O,'--r');
% title('Oversteering')
% xlabel('side slip (rad)')
% ylabel('Norm. lat. accel. ()')
% 
% subplot(5,3,15)
% plot(res_par_U.alphaRVec,res_par_U.fyRVec,'xb'); hold on
% plot(ssa,Fr_U,'--b');
% plot(res_par_U.alphaFVec,res_par_U.fyFVec,'or'); 
% plot(ssa,Ff_U,'--r');
% title('Understeering')
% xlabel('side slip (rad)')
% ylabel('Norm. lat. accel. ()')
% 
% 
% set(gcf,'PaperPositionMode','auto')
% %print('-depsc','-tiff','-r300',fig_name)
% print('-dpdf','-r300',fig_name)
% %%
% % figure
% % plot(U0vec,U0vec-res_par_N.VvecTrue,'-xg'); hold on
% % plot(U0vec,U0vec-res_par_O.VvecTrue,'-xr')
% % plot(U0vec,U0vec-res_par_U.VvecTrue,'-xb')
%% Comparison
figure()
plot(0:Ts:(length(STEP_STEER.delta_HW)-1)*Ts,STEP_STEER.yaw_rate*180/pi)
hold on
plot(time1,Omega*180/pi,'LineWidth',2)

figure()
plot(0:Ts:(length(STEP_STEER.delta_HW)-1)*Ts,STEP_STEER.long_vel.*3.6)
hold on
plot(time1,u*3.6,'LineWidth',2);
