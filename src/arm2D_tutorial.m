function arm2D_tutorial()
%% Grid
% TODO should this be from 0 to 2pi for all links or joint limits?
grid_min = [0; -pi];          % Lower corner of computation domain
grid_max = [pi; pi];      % Upper corner of computation domain
N = [81;81];                 % Number of grid points per dimension
pdDims = 2;                 % 2nd dimension is periodic (1st goes from 0 to pi only)
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% robot arm
% Define dynamic system
xinit = [pi/2, 0];
% input bounds
uMin = [-1, -1];
uMax = [1, 1];
% link properties
l1 = 1;
l2 = 1;
dims = 1:2;
% do dStep1 here
% obj = DubinsCar(x, wMax, speed, dMax)
arm = Arm2D(xinit, uMin, uMax, dims, l1, l2, grid_min, grid_max); %do dStep3 here

%% target set (avoid)
% R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
% data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% center = [pi/2, pi, 0, 0];
% R = pi/2;
% data0 = shapeSphere(g,center,R);
data0 = -shapeGroundPlane(g, arm);

%% time vector
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% control trying to min or max value function?
uMode = 'min';
% do dStep2 here

%% Pack problem parameters

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = arm;
schemeData.accuracy = 'low'; %set accuracy
schemeData.uMode = uMode;
%do dStep4 here


%% If you have obstacles, compute them here

%% Compute value function

HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'maxVOverTime', HJIextraArgs);

save('/home/andreabajcsy/hybrid_ws/src/rawr/matfiles/arm2DpiN81.mat')

compTraj = false;

%% Compute optimal trajectory from some initial state
if compTraj
  pause
  
  %set the initial state
  xinit = [2, 1, -pi];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    arm.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,4);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, arm, TrajextraArgs);
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
end

function data = shapeGroundPlane(grid, arm)
    % Input Parameters:
    %
    %   grid: Grid structure (see processGrid.m for details).
    %   arm: Arm4D object (for forward kinematics).
    %
    % Output Parameters:
    %
    %   data: Output data array (of size grid.size) containing the implicit
    %   surface function.
    
    % Signed distance function calculation.
    data = zeros(grid.shape);
    th1_range = linspace(grid.min(1), grid.max(1), grid.shape(1));
    th2_range = linspace(grid.min(2), grid.max(2), grid.shape(2));  
    
    i = 1; 
    for th1 = th1_range
        j = 1;
        for th2 = th2_range
            [pos_elbow, pos_ee] = arm.fwd_kinematics([th1; th2]);
            % get the minimum y-distance
            data(i,j,:,:) = min(pos_elbow(2), pos_ee(2));
            j = j+1;
        end
        i = i+1;
    end

    % Warn the user if there is no sign change on the grid
    %  (ie there will be no implicit surface to visualize).
    if(all(data(:) < 0) || (all(data(:) > 0)))
      warning([ 'Implicit surface not visible because function has ' ...
                'single sign on grid' ]);
    end
end