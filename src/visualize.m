load('armN81.mat');
grid_min = [0; 0; -1; -1]; % Lower corner of computation domain
grid_max = [pi; 2*pi; 1; 1];    % Upper corner of computation domain
N = [11;11;11;11];         % Number of grid points per dimension
pdDims = [1; 2];               % 1st and 2nd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

% control trying to min or max value function?
uMode = 'min';

% Define dynamic system
xinit = [pi/2, pi/2, 0, 0];
% input bounds
uMin = [-3, -3];
uMax = [3, 3];
% link properties
l1 = 1;
l2 = 1;
m1 = 0.5;
m2 = 0.5;
dims = 1:4;
% do dStep1 here
% obj = DubinsCar(x, wMax, speed, dMax)
arm = Arm4D(xinit, uMin, uMax, dims, l1, l2, m1, m2, grid_min, grid_max); %do dStep3 here

compTraj = true;

%% Compute optimal trajectory from some initial state
if compTraj
  pause 
  
  %set the initial state
  xinit = [pi/2, 0, -0.2, 0];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    arm.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,5);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, arm, TrajextraArgs);
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end