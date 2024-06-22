clear all
close all
clc

% fileName = 'fennLanesEB_X180m280m_Y1m3p5m'; % used for ACC 2024
% fileName = 'potBumpTestCase13';
fileName = 'potBumpTestCase16';
% fileName = 'testCase0';

%% generate XY Plan
[bicycleStatesXYTheta, steeringAll, arcLengthAll, params, totalTime] = getXYPlan(fileName);

%% generate velocity, acceleration, and time profiles
[timeTakenAll, velocitiesAll, accelerationsAll] = getVATProfiles(arcLengthAll, params);

%% collect everything in the params structure 
params.xyPlan.timeTakenAll = timeTakenAll;
params.xyPlan.bicycleStatesXYTheta =bicycleStatesXYTheta;
params.xyPlan.velocitiesAll = velocitiesAll;
params.xyPlan.steeringAll = steeringAll;
params.xyPlan.accelerationsAll = accelerationsAll;

%% perform z planning
[minIndxZ, z, susForce, searchTimeZ, totalCostZ] = getZPlan(params);

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
%     'searchTimeZ', 'minIndxZ', 'velocitiesAll', 'accelerationsAll', 'timeTakenAll', 'totalCostZ')