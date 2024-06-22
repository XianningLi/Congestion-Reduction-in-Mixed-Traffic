clear all
close all
clc

% fileName = 'fennLanesEB_X180m280m_Y1m3p5m'; % used for ACC 2024
% fileName = 'potBumpTestCase13';
fileName = 'potBumpTestCase16';
% fileName = 'testCase0';

% run the parameter file
params = loadParams(strcat(fileName,'.json'));

Zest = params.road.RBFBasis*params.road.alpha;  % compute estimated road profile
Zest = reshape(Zest, round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));
params.road.Zest = Zest;

x = params.road.X(1,:);
y = params.road.Y(:,1);

% subsampling X, Y, Z according to the planning horizon
Xn = params.road.X(1:params.xyPlan.subSampleY:numel(y), 1:params.xyPlan.subSampleX:numel(x));
Yn = params.road.Y(1:params.xyPlan.subSampleY:numel(y), 1:params.xyPlan.subSampleX:numel(x));
Zn = params.road.Z(1:params.xyPlan.subSampleY:numel(y), 1:params.xyPlan.subSampleX:numel(x));

% the initial pose
bestPrev = [params.veh.initialVals(1) params.veh.initialVals(2) params.veh.initialVals(3) 0]';

% compute lane center
laneCenter = (Yn(end,1)-Yn(1,1))/2;

% choose the point in the grid that is closest to the laneCenter
[~,laneCenterIndx] = min(abs(Yn(:,1)-laneCenter));
laneCenter = Yn(laneCenterIndx,1);

% add a few more components to the XY plan params
params.xyPlan.ds = params.xyPlan.subSampleX*params.road.gridSize; % lookahead
params.xyPlan.X = Xn;
params.xyPlan.Y = Yn;
params.xyPlan.Z = Zn;
params.xyPlan.laneCenter = laneCenter;
params.flag = "XY";

% initial guess for the solver
p10 = [0 0 params.veh.initialVals(1)]';
p0 = p10;

% some initializations
bestPrevAll = zeros(numel(bestPrev), size(Xn,2)-1);
bestPAll = zeros(numel(p10), size(Xn,2)-1);

bestPrevAll(:,1) = bestPrev;
bestPAll(:,1) = p10;

% v0 = params.veh.initialVals(4); % initial velocity

bicycleStatesXYTheta = [bestPrev(1:3)];
steeringAll = [bestPrev(4)];
arcLengthAll = [p10(end)];

time0 = 0; % initial time
zAll = [params.veh.initialVals(5:end)];
susForceAll = [zeros(4,1)];
velocitiesAll = [];
timeTakenAll = [];
accelAll = [0];

totalTime = 0;
count1 = 2;
count2 = params.xyPlan.numSamplesPerSegment+1;

% select the type of spiral: please only use cubic. Quartic and quitintic are not fully implemented
flag = "cubic";
% flag = "quartic";

x = linspace(0, params.road.Nx, round(params.road.Nx/(0.1)));   % select x positions for basis
y = linspace(0, params.road.Ny, round(params.road.Ny/(0.1)));   % select y positions for basis
[X, Y] = meshgrid(x, y);
[Xq,Yq,Zq] = meshgrid(0:0.01:100,0:0.01:3.7,min(params.road.Z):0.01:max(params.road.Z));
vq = griddata(X,Y,params.road.Z,Xq,Yq);

% start plotting
figure
% mesh(params.road.X, params.road.Y, roadFunPot(params.road.X,params.road.Y)+roadFunBump(params.road.X,params.road.Y))
% mesh(params.road.X, params.road.Y, params.road.Z)
mesh(Xq, Yq, vq)
daspect([1 1 1])
% hold on
% plot(bestPrev(1), bestPrev(2), 'ro')
hold on
% [cylX, cylY, cylZ] = cylinder(params.xyPlan.obsRadi);
cylX = params.xyPlan.obsX + params.xyPlan.obsRadi*sin(0:0.1:2*pi);
cylY = params.xyPlan.obsY + params.xyPlan.obsRadi*cos(0:0.1:2*pi);
cylZ = 0.6*ones(size(cylX));
fill3(cylX, cylY, cylZ, 'm')
% colormap winter;

i = 2;
targets = 0;
while targets <= Xn(1,end) % perform conformal lattice planning

    % compute the next target poses
    targets = getTargets(params, bestPrev);

    % get the updated spiral parameters for each target poses (updatedP is a matrix)
    [solTime, updatedP] = getUpdatedPOpti(repmat(p10,1,size(targets,2)), params, targets, bestPrev);

    % get the best target pose to connect from the initial pose and the corresponding best spiral parameters
    tic
    [bestPoint, bestP] = getBestPoint(params, updatedP, bestPrev, targets);
    searchTimeXY = toc;

    % store the best poses and the corresponding spiral parameters
    bestPrevAll(:,i) = bestPoint;
    bestPAll(:,i) = bestP;

    %more plotting
    hold on
    plot(targets(1,:), targets(2,:), 'bo')
    hold on
    plot(bestPoint(1), bestPoint(2), 'ro')
    for k = 1:size(updatedP,2)
        hold on
        bicycleStates = computeKinematics(updatedP(:,k), 0:0.01:updatedP(end,k), bestPrev, params);
        plot3(bicycleStates(1,:), bicycleStates(2,:), 0.1*ones(size(bicycleStates(2,:))), 'y', 'linewidth', 2)
    end
    hold on

    % compute [x, y, theta, kappa] for the best pose
    bicycleStates = computeKinematics(bestP, linspace(0, bestP(end), params.xyPlan.numSamplesPerSegment), bestPrev, params);

    % store the states [x, y, theta]
    bicycleStatesXYTheta = [bicycleStatesXYTheta bicycleStates(1:3,2:end)];

    % store the steering kappa
    steeringAll = [steeringAll bicycleStates(4,2:end)];

    % store arc length
    updateArcLength = arcLengthAll(end)+linspace(0, bestP(end), params.xyPlan.numSamplesPerSegment);
    arcLengthAll = [arcLengthAll  updateArcLength(2:end)]; % continuation of arc length

    % compute the total computation time
    totalTime = totalTime + solTime + searchTimeXY;

    bestPrev = [bicycleStates(1:3,end); bicycleStates(4,end)];
    p0 = bestP;

    count1 = count1 + params.xyPlan.numSamplesPerSegment;
    count2 = count2 + params.xyPlan.numSamplesPerSegment;
    i = i+1;
end

%% generate velocity, acceleration, and time profiles
[timeTakenAll, velocitiesAll, accelerationsAll] = getVATProfiles(arcLengthAll);

%% collect everything in the params structure 
params.xyPlan.timeTakenAll = timeTakenAll;
params.xyPlan.bicycleStatesXYTheta =bicycleStatesXYTheta;
params.xyPlan.velocitiesAll = velocitiesAll;
params.xyPlan.steeringAll = steeringAll;
params.xyPlan.accelerationsAll = accelerationsAll;

%% perform z planning
[minIndxZ, z, susForce, searchTimeZ] = getZPlan(params);

%% Arrange
zBest = z(:, :, minIndxZ);
susForceBest = susForce(:, :, minIndxZ);
timeTakenBest = params.xyPlan.timeTakenAll(minIndxZ, :);

x = [params.xyPlan.bicycleStatesXYTheta; params.xyPlan.velocitiesAll(minIndxZ, :); zBest];
u = [steeringAll(1:end-1); accelerationsAll(minIndxZ, 1:end-1); susForceBest];

%% animation
% plotTraj(x, u, params, 0, strcat(fileName,'Demo'))

%% save the plan
% save(strcat('D:\Sayan Github\research-2023-motion-planning\matFiles\', fileName, 'LatticeXYZPlan_ACC2024' ,'.mat'), 'x', 'u', 'timeTakenBest', 'totalTime', ...
%     'searchTimeZ', 'minIndxZ', 'velocitiesAll', 'accelerationsAll', 'timeTakenAll', 'totalCostZ', 'totalTime')







%% add noise to road height
% fileName = strcat('potBumpNoiseTestCase13.json');
% s.X = params.road.X;
% s.Y = params.road.Y;
% s.Z = params.road.Z + 0.01*rand(round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));
% s.label = params.road.label;
% s.gridSize = params.road.gridSize;
% s.Nx = params.road.Nx;
% s.Ny = params.road.Ny;
% % s.dimLoc = params.road.dimLoc;
% s.initX = params.road.initX;
% s.decayRate = params.road.decayRate;
% s.space = params.road.space;
% writeScenarioToJson(s, fileName)
% 
% params = loadParams(strcat(fileName));
% Zest = params.road.RBFBasis*params.road.alpha;  % compute estimated road profile
% Zest = reshape(Zest, round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));
% params.road.Zest = Zest;