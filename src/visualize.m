load('/home/andreabajcsy/hybrid_ws/src/rawr/matfiles/arm2DpiN81.mat');

%% Compute optimal trajectory from some initial state

%pause 

%set the initial state
xinit = [pi/2, pi/2];

%check if this initial state is in the BRS/BRT
%value = eval_u(g, data, x)
value = eval_u(g,data(:,:,end),xinit);

%if value <= 0 %if initial state is in BRS/BRT
% find optimal trajectory

arm.x = xinit; %set initial state of the dubins car

TrajextraArgs.uMode = uMode; %set if control wants to min or max
TrajextraArgs.visualize = true; %show plot
TrajextraArgs.fig_num = 2; %figure number

%we want to see the first two dimensions (x and y)
TrajextraArgs.projDim = [1 1]; 

%flip data time points so we start from the beginning of time
dataTraj = flip(data,3);

% [traj, traj_tau] = ...
% computeOptTraj(g, data, tau, dynSys, extraArgs)
[traj, traj_tau] = ...
  computeOptTraj(g, dataTraj, tau2, arm, TrajextraArgs);

filename = '2Darmpi3.gif';
arm.plot_traj(traj, 0.05, filename);
%else
%error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
%end

