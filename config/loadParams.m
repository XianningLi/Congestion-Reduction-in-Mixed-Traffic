function params = loadParams(scenarioFilename)

if nargin < 1
    scenarioFilename = 'testcase0.json';
end

try
    testScenario = readScenarioFromJson(scenarioFilename);
catch ex
    warning("Failed to load scenario file ... reverting to the default scenario ... ")
    scenarioFilename = 'testcase0.json';
    testScenario = readScenarioFromJson(scenarioFilename);
end

%% vehicle parameters
params.veh.m = 1000;                       % Mass [kg]
params.veh.b = 4000;                       % Damping constant [N.s/m]
params.veh.Jb = 2400;                      % Pitch moment of inertia [kg m^2]
params.veh.Jg = 400;                       % Yaw moment of inertia [kg m^2]
params.veh.l1 = 1.15;                      % Front length of car [m]
params.veh.l2 = 1.56;                      % Back length of car [m]
params.veh.w  = 0.5;                       % Width of car [m] (w/2)
params.veh.k  = [36297 36297 36297 36297]; % Spring constant [N/m]
params.veh.c  = [3924 3924 3924 3924];     % Damping constant [N.s/m]

% suspension travel limit
params.veh.susTravLim = [-0.1 0.1
                         -0.1 0.1 
                         -0.1 0.1 
                         -0.1 0.1];
% vehicle initial condition
params.veh.initialVals = testScenario.initX;
% params.veh.initialVals(2) = 1; % have to change this
params.veh.initialVals(4) = 10;
params.veh.initialVals(5) = 0;

%% road surface parameters
params.road.label = testScenario.label;
% params.road.dimLoc = testScenario.dimLoc;
params.road.space = testScenario.space;
params.road.initX = testScenario.initX;
% generate road data
params.road.Nx = testScenario.Nx; % length of road in x direction (m)
params.road.Ny = testScenario.Ny; % length of road in y direction (m)
params.road.gridSize = testScenario.gridSize; % 10 cm
params.road.decayRate = [testScenario.decayRate(1) testScenario.decayRate(2)];
params.road.X = testScenario.X;
params.road.Y = testScenario.Y;
params.road.Z = testScenario.Z;

xb = linspace(1, params.road.Nx, round(params.road.Nx/(testScenario.space(1)*params.road.gridSize)));   % select x positions for basis
yb = linspace(1, params.road.Ny, round(params.road.Ny/(testScenario.space(2)*params.road.gridSize)));   % select y positions for basis

[params.road.xxb, params.road.yyb] = meshgrid(xb, yb);

[params.road.alpha, params.road.RBFBasis] = getAlpha(params);
RoadData = approximateRoad(params);
params.road.RoadData = RoadData; % Road Profile data


%% opitmization and objective function parameters
params.opti.N = 60;  		       % Horizon
% weigths to penalize states and inputs in objective function are arranged
% in this order [xN yN x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3 u4 u5 u6 heaveAcc pitchAcc rollAcc]
%params.opti.weights = [0 0 0 1 0.01 0.01 0 0 0 0 2 1 0 0 0 0 5 5 5];

params.opti.weights.cost_hpr_acc  = [5 5 5];
params.opti.weights.cost_fcmd     = 0*[1 1 1 1];
params.opti.weights.cost_hpr_disp = [5 5 5];
params.opti.weights.cost_hpr_vel  = [0.01 0 0];
params.opti.weights.cost_lat      = [5 100 70];
params.opti.weights.cost_long     = [0.4 1];
params.opti.dt = 0.1; % sampling interval for RK4
params.opti.vref = 26; % reference velocity (m/s)
params.opti.vMin = 0.1;
params.opti.XY = [1.2480]; % set of target positions
%% control limts
params.ctrlLims.uLim = [-0.19 0.19 % steering
                        -2 2       % acceleration
                        -2000 2000 % faFL
                        -2000 2000 % faFR
                        -2000 2000 % faRL
                        -2000 2000]; % faRR

%% state limits
params.stateLims.xLim = [0  params.road.Nx;
    0  params.road.Ny;
    -inf inf;
    0.1 inf;
    -inf inf;
    -inf inf;
    -inf inf;
    -inf inf;
    -inf inf;
    -inf inf];

%% visualization params
params.viz.steps = 8;
params.viz.steadyStateHeight = 1.0; % (m)
params.viz.wheelRadius = 0.2;
params.viz.wheelWidth = 0.15;

%% XY planner params 

% parameters for subsampling
params.xyPlan.subSampleX = 100; % each grid is 0.1 meter, thus the planning horizon is gridsize*params.XYPlan.subSampleX
params.xyPlan.subSampleY = 1; 
params.xyPlan.numSamplesPerSegment = 20; % specifies the discetization for solutions in each lookahead

% parameters for optimizing spirals
params.xyPlan.weights_cost_rxy = [6000 6000 6000 6000]; % penalizes road disturbances at four corners
params.xyPlan.weights_cost_obs = 1e4;
params.xyPlan.weights_spiral = [100 100 100]; % penalizes deviation from target poses
params.xyPlan.weigths_laneCenter = 100; % penalizes deviation from lane center

% parametrs for obstacles
params.xyPlan.obsX = 27.5;
params.xyPlan.obsY = 1;
params.xyPlan.obsRadi = 0.3;
% params.xyPlan.obsX = 0;
% params.xyPlan.obsY = 0;
% params.xyPlan.obsRadi = 0;

% params for selecting the best z plan 
params.xyPlan.velWeigth = 1;
params.xyPlan.zCostWeigth = 1;
end
