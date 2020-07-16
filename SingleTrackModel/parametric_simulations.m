res_par.VvecTrue  = zeros(size(U0vec));
res_par.OmegaVec  = zeros(size(U0vec));
res_par.BetaVec   = zeros(size(U0vec));
res_par.alphaRVec = zeros(size(U0vec));
res_par.alphaFVec = zeros(size(U0vec));
res_par.fyRVec    = zeros(size(U0vec));
res_par.fyFVec    = zeros(size(U0vec));
res_par.Rvec      = zeros(size(U0vec));
res_par.RvecTrue  = zeros(size(U0vec));
res_par.FXRvec    = zeros(size(U0vec));
res_par.FXFvec    = zeros(size(U0vec));

for i =1:length(U0vec)
  u0 = U0vec(i);
  U0 = u0; % Desired forward velocity
  % Set the state initial condition using a structure (to avoid warning)
  xInitial1 = [0, 0, 0, u0, 0, 0]; % velocities
  init_states1.time = 0;
  init_states1.signals.values  =  xInitial1;
  init_states1.signals.label   = 'CSTATE';
  init_states1.signals.blockName  ='single_track/Integrator';
  set_param(cs1,'LoadInitialState' ,'on');
  set_param(cs1,'InitialState','init_states1');
  
  %Simulate the model
  fprintf(1,'Simulating model u0 = %6.3f -- [Kyr, Kyf] = [%6.3f , %6.3f]\n', ...
    u0,data.Kyr, data.Kyf)
  simOut1 = sim(mdl1,cs1);
  %extract data
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
  u       = res1(:,4);
  v       = res1(:,5);
  Omega   = res1(:,6);
  Fzr     = outputs(:,1);
  Fzf     = outputs(:,2);
  alpha_r = outputs(:,3);
  alpha_f = outputs(:,4);
  Fyr     = outputs(:,5);
  Fyf     = outputs(:,6);
  
  gcf;
  clf
  subplot(5,1,1); cla
  plot(time1,u*3.6,'LineWidth',2); hold on
  plot(time1,v*3.6,'-r','LineWidth',2)
  title('Velocity')
  legend('u','v')
  ylim([-20 160])
  
  subplot(5,1,2); cla
  plot(time1,Omega*180/pi,'LineWidth',2); hold on
  title('Yaw rate')
  ylabel('(deg)')
  
  subplot(5,1,3); cla
  plot(x,y,'LineWidth',2); hold on
  title('trajectory')
  axis equal
  
  subplot(5,1,4); cla
  plot(time1,Omega.*u/9.81,'LineWidth',2); hold on
  title('lateral acceleration')
  ylim([-1 1])
  
  subplot(5,1,5); cla
  plot(time1,Fxr,'LineWidth',2); hold on
  plot(time1,Fxf,'-r','LineWidth',2)
  title('longitudinal force')
  legend('F_{xr}','F_{xf}')

  drawnow
  VG = sqrt(u0^2+v(end)^2);
  res_par.VvecTrue(i)  = VG;
  res_par.OmegaVec(i)  = Omega(end);
  res_par.BetaVec(i)   = atan(v(end)/u0);
  res_par.alphaRVec(i) = alpha_r(end);
  res_par.alphaFVec(i) = alpha_f(end);
  res_par.fyRVec(i)    = Fyr(end)/Fzr(end);
  res_par.fyFVec(i)    = Fyf(end)/Fzf(end);
  res_par.Rvec(i)      = u0/res_par.OmegaVec(i);
  res_par.RvecTrue(i)  = VG/res_par.OmegaVec(i);
  res_par.FXRvec(i)    = Fxr(end);
  res_par.FXFvec(i)    = Fxf(end);
  %fprintf(1,'%6.3f %6.3f(%6.3f)-%6.3f\n',u0,Omega(end),res_par.OmegaVec(i),res_par.Rvec(i))
end