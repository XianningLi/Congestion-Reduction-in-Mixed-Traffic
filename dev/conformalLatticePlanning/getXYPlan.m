function [bicycleStatesXYTheta, steeringAll, arcLengthAll, params, totalTime] = getXYPlan(fileName)

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

bicycleStatesXYTheta = [bestPrev(1:3)];
steeringAll = [bestPrev(4)];
arcLengthAll = [p10(end)];

time0 = 0; % initial time
zAll = [params.veh.initialVals(5:end)];
susForceAll = [zeros(4,1)];
accelAll = [0];

totalTime = 0;
count1 = 2;
count2 = params.xyPlan.numSamplesPerSegment+1;

% select the type of spiral: please only use cubic. Quartic and quitintic are not fully implemented
flag = "cubic";
% flag = "quartic";

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

    for k = 1:size(updatedP,2)
        bicycleStates = computeKinematics(updatedP(:,k), 0:0.01:updatedP(end,k), bestPrev, params);
    end

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

end