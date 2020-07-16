%% MODELLING AND SIMULATION OF A SINGLE TRACK MODEL
close all
clear all
clc


% All the system parameters are stred in a vector in order to easely pass the 
% parameters to the Simulink model.
% Model parameters
data.gravity = 9.81;
data.mass    = 1000.0;
data.Iz      = 1200.0;
data.Lr      = 1.8;
data.Lf      = 3.0-1.8;
data.kv      = 0.0;
data.h       = 0.5;

% tyre parameters  %     
data.Byr     = 5.0; %7.0
data.Cyr     = 2.0;
data.Dyr     = 1.0;
data.Eyr     = 0.9;
data.Kyr     = data.Byr*data.Cyr*data.Dyr;

data.Byf     = 7.0;
data.Cyf     = 2.0;
data.Dyf     = 0.95;
data.Eyf     = 0.9;
data.Kyf     = data.Byf*data.Cyf*data.Dyf;

data.tyre_model = 1; %'linear';
data.tyre_model = 2; %'non_linear';


figure()
alpha = [-0.5:0.01:0.5];
FYR    = magic_formula(alpha,data.Byr,data.Cyr,data.Dyr,data.Eyr);
FYF    = magic_formula(alpha,data.Byf,data.Cyf,data.Dyf,data.Eyf);
plot(alpha,FYR); hold on
plot(alpha,FYF,'--r','LineWidth',2)
legend('rear','front')



% Parameters used in the step blocks
U0     = 95.0/3.6; % Desired forward velocity
kp     = 500;      % Gain P controller for velocity tracking
delta0 = 0.01;     % steer step
t0     = 5;        % time step begin step

fig_name = ['./figures/fig_dyn_characteristics_kv',num2str(data.kv),'.pdf'];
%% Model without relaxation length
% Load model without tyre dynamics and run the simulation
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
set_param(cs1,'StopTime'          , '8.0');
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
xInitial1 = [0, 0, 0,  ... %positions
             U0, 0, 0]; % velocities
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
