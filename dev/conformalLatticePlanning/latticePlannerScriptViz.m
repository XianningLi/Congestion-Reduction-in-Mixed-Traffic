clear all
close all
clc

fileName = 'fennLanesEB_X180m280m_Y1m3p5m';
% fileName = 'potBumpTestCase18';
% fileName = 'testCase0';

% run the parameter file
params = loadParams(strcat(fileName,'.json')); 

x = params.road.X(1,:); 
y = params.road.Y(:,1);
subSampleX = params.xyPlan.subSampleX; 
subSampleY = params.xyPlan.subSampleY; 

% subsampling X, Y, Z according to the planning horizon
Xn = params.road.X(1:subSampleY:numel(y), 1:subSampleX:numel(x));
Yn = params.road.Y(1:subSampleY:numel(y), 1:subSampleX:numel(x));
Zn = params.road.Z(1:subSampleY:numel(y), 1:subSampleX:numel(x));

% the initial pose
bestPrev = [Xn(1,1) params.opti.XY 0 0]';

% compute lane center
laneCenter = (Yn(end,1)-Yn(1,1))/2;

% choose the point in the grid that is closest to the laneCenter
[~,laneCenterIndx] = min(abs(Yn(:,1)-laneCenter));
laneCenter = Yn(laneCenterIndx,1);

% add a few more components to the XY plan params
params.xyPlan.ds = subSampleX*params.road.gridSize; % lookahead
params.xyPlan.X = Xn;
params.xyPlan.Y = Yn;
params.xyPlan.Z = Zn;
params.xyPlan.laneCenter = laneCenter;

% initial guess for the solver
p10 = [0 0 0.001]';
p0 = p10;
% p0 = [0
%     0.740388246402895
% -0.267887888696431
% 0.239759645996300
% 0.1
% 2.31475609138151];

% p0 = [0
%     0.740388246402895
%     0.123563243178735
%     -0.239759645996302
%     0];

% some initializations
bestPrevAll = zeros(numel(bestPrev), size(Xn,2));
bestPAll = zeros(numel(p10), size(Xn,2));

bestPrevAll(:,1) = bestPrev;
bestPAll(:,1) = p10;

% bicycleStatesXYTheta = zeros(3, params.XYPlan.numSamplesPerSegment*(size(Xn,2)-1)+1);
% steeringAll = zeros(1, params.XYPlan.numSamplesPerSegment*(size(Xn,2)-1)+1);

bicycleStatesXYTheta = [bestPrev(1:3)];
steeringAll = [bestPrev(4)];
arcLengthAll = [p10(end)];

totalTime = 0;
count1 = 2;
count2 = params.xyPlan.numSamplesPerSegment+1;

Zest = params.road.RBFBasis*params.road.alpha;  % compute estimated road profile
Zest = reshape(Zest, round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));

% select the type of spiral: please only use cubic. Quartic and quitintic are not fully implemented
flag = "cubic"; 
% flag = "quartic"; 

% start plotting
figure
mesh(params.road.X, params.road.Y, Zest)
daspect([1 1 1])
hold on
plot(bestPrev(1), bestPrev(2), 'ro')
hold on
circX = params.xyPlan.obsX + params.xyPlan.obsRadi*cos(0:0.1:2*pi);
circY = params.xyPlan.obsY + params.xyPlan.obsRadi*sin(0:0.1:2*pi);
circZ = 0.06*ones(1, numel(circY));
fill3(circX, circY, circZ, 'm')

i = 2;
targets = 0;
while targets <= Xn(1,end-1) % perform conformal lattice planning

    % compute the next target poses
    targets = getTargets(params, bestPrev);

    % get the updated spiral parameters for each target poses (updatedP is a matrix)
    [solTime, updatedP] = getUpdatedPOpti(repmat(p10,1,size(targets,2)), params, targets, bestPrev, flag);

    % get the best target pose to connect from the initial pose and the corresponding best spiral parameters
    tic
    [bestPoint, pNew] = getBestPoint(params, updatedP, bestPrev, targets, flag);
    searchTime = toc;

    % compute the total computation time
    totalTime = totalTime + solTime + searchTime;

    % store the best poses and the corresponding spiral parameters
    bestPrevAll(:,i) = bestPoint;
    bestPAll(:,i) = pNew;

    % more plotting
    hold on
    plot(targets(1,:), targets(2,:), 'bo')
    hold on
    plot(bestPoint(1), bestPoint(2), 'ro')
    for j = 1:size(updatedP,2)
        hold on
        bicycleStates = computeKinematics(updatedP(:,j), 0:0.01:updatedP(end,j), bestPrev, flag);
        plot(bicycleStates(1,:), bicycleStates(2,:), 'b', 'linewidth', 2)
    end
    hold on
%     bicycleStates = computeKinematics(pNew, 0:0.01:pNew(end), bestPrev);
    
    % compute [x, y, theta, kappa] for the best pose 
    bicycleStates = computeKinematics(pNew, linspace(0, pNew(end), params.xyPlan.numSamplesPerSegment), bestPrev, flag);
%     bicycleStates = computeKinematics(pNew, 0:0.5:pNew(end)/2, bestPrev, flag);

    % store the states [x, y, theta]
    bicycleStatesXYTheta = [bicycleStatesXYTheta bicycleStates(1:3,2:end)];

    % store the steering kappa
    steeringAll = [steeringAll bicycleStates(4,2:end)];

    % store arc length
    updateArcLength = arcLengthAll(end)+linspace(0, pNew(end), params.xyPlan.numSamplesPerSegment);
    arcLengthAll = [arcLengthAll  updateArcLength(2:end)]; % continuation of arc length

    bestPrev = [bicycleStates(1:3,end); bicycleStates(4,end)];
%     bestPrev = bestPoint;

    p0 = pNew;

    count1 = count1 + params.xyPlan.numSamplesPerSegment;
    count2 = count2 + params.xyPlan.numSamplesPerSegment;
    i = i+1;
end

%% Visualization
hold on
plot(bicycleStatesXYTheta(1,:), bicycleStatesXYTheta(2,:), 'r', 'linewidth', 5)
xlabel('x')
ylabel('y')

figure
plot(bicycleStatesXYTheta(1,:), bicycleStatesXYTheta(2,:), 'r', 'linewidth', 2)
xlabel('x')
ylabel('y')
axis equal

figure
plot(bicycleStatesXYTheta(3,:))
hold on
plot(steeringAll)
legend('\theta', '\kappa')
%% save the plan
x = bicycleStatesXYTheta;
u = steeringAll;
s = arcLengthAll;
save(strcat('C:\git\research-2023-motion-planning\matFiles\', fileName, 'LatticeXYPlan' ,'.mat'), "x", "u", "s")