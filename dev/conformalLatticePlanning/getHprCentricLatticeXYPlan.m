function bestXYPlan = getHprCentricLatticeXYPlan(fileName)
% this function is under development

% run the parameter file
params = loadParams(strcat(fileName,'.json')); 

x = params.road.X(1,:);
y = params.road.Y(:,1);
subSampleX = params.XYPlan.subSampleX; 
subSampleY = params.XYPlan.subSampleY; 

% subsampling X, Y, Z according to the planning horizon
Xn = params.road.X(1:subSampleY:numel(y), 1:subSampleX:numel(x));
Yn = params.road.Y(1:subSampleY:numel(y), 1:subSampleX:numel(x));
Zn = params.road.Z(1:subSampleY:numel(y), 1:subSampleX:numel(x));

params.latticePlanner.ds = subSampleX*params.road.gridSize; % planning horizon
params.latticePlanner.X = Xn;
params.latticePlanner.Y = Yn;
params.latticePlanner.Z = Zn;
params.latticePlanner.laneCenter = (Yn(end,1)-Yn(1,1))/2;

% the initial best pose
bestPrev = [Xn(1,1) params.opti.XY 0 0]';

% initial guess for the solver
p0 = [0 1 1 1 0.00001]';
% p0 = [0
%     0.740388246402895
% -0.267887888696431
% 0.239759645996300
% 2.31475609138151];
% p0 = [0
%     0.740388246402895
%     0.123563243178735
%     -0.239759645996302
%     0];

% some initializations
bestPrevAll = zeros(numel(bestPrev), size(Xn,2));
bestPAll = zeros(numel(p0), size(Xn,2));

bestPrevAll(:,1) = bestPrev;
bestPAll(:,1) = p0;

bicycleStatesXYTheta = zeros(3, params.XYPlan.numSamplesPerSegment*(size(Xn,2)-1)+1);
steering = zeros(1, params.XYPlan.numSamplesPerSegment*(size(Xn,2)-1)+1);

bicycleStatesXYTheta(:,1) = bestPrev(1:3);
steering(:,1) = bestPrev(4);

totalComputeTime = 0;
count1 = 2;
count2 = params.XYPlan.numSamplesPerSegment+1;

for i = 2:size(Xn,2) % perform conformal lattice planning

    % compute the next target poses
    targets = getTargets(params, bestPrev, i);

    % get the updated spiral parameters for each target poses (updatedP is a matrix)
    [solTime, updatedP] = getUpdatedPOpti(repmat(p0,1,size(targets,2)), targets, bestPrev); 

    % get the best target pose to connect from the initial pose and the corresponding best spiral parameters
    tic
    [bestPoint, pNew] = getBestPoint(params, updatedP, bestPrev, targets); 
    searchTime = toc;

    % compute the total computation time
    totalComputeTime = totalComputeTime + solTime + searchTime;

    % store the best poses and the corresponding spiral parameters
    bestPrevAll(:,i) = bestPoint;
    bestPAll(:,i) = pNew;

    % compute [x, y, theta, kappa] for the best pose 
    bicycleStates = computeKinematics(pNew, linspace(0, pNew(end), params.XYPlan.numSamplesPerSegment ), bestPrev);

    % store the states [x, y, theta]
    bicycleStatesXYTheta(:,count1:count2) = bicycleStates(1:3,:);

    % store the steering kappa
    steering(:,count1:count2) = bicycleStates(4,:);

    bestPrev = bestPoint;
    p0 = pNew;

    count1 = count1 + params.XYPlan.numSamplesPerSegment;
    count2 = count2 + params.XYPlan.numSamplesPerSegment;
end

% return the best plan and total computation time
bestXYPlan.XYTheta = bicycleStatesXYTheta;
bestXYPlan.steering = steering;
bestXYPlan.totalComputeTime = totalComputeTime;

end