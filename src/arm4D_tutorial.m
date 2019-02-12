function arm4D_tutorial()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;

%% Grid
% TODO should this be from 0 to 2pi for all links or joint limits?
grid_min = [0; 0; -1; -1]; % Lower corner of computation domain
grid_max = [pi; 2*pi; 1; 1];    % Upper corner of computation domain
N = [11;11;11;11];         % Number of grid points per dimension
pdDims = 2;               % 2nd dimension is periodic (1st goes from 0 to pi only)
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% robot arm
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

%% target set (avoid)
% R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
% data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% center = [pi/2, pi, 0, 0];
% R = pi/2;
% data0 = shapeSphere(g,center,R);
data0 = -shapeGroundPlane(g, arm); % this is your l(x)

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

%% Compute value function

HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
minWith = 'maxVOverTime';

% data0 = l(x) so that you start your computation at the final time
% with value: V(x,T) = l(x) = data0
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);

save('/home/andreabajcsy/hybrid_ws/src/rawr/matfiles/armN11_2.mat')

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