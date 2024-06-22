function targets = getTargets(params, bestPrev)

% y targets are selected as all the posible y points in the subsampled grid
yTargets = params.xyPlan.Y(6:end-6,1); % reducing some points from the road edges so that car does not hit the curbside 

% x targets are selected at each planning horizon
xTargets = bestPrev(1) + params.xyPlan.ds;

% compute theta
% th = atan(yDist/xDist);
th = zeros(numel(yTargets),1);

% compute kappa
% kappa = (th - bestPrev(end))/params.latticePlanner.ds;
kappa = zeros(numel(yTargets),1);

% return targets
targets = [ones(1,numel(yTargets))*xTargets
    yTargets'
    th'
    kappa'];

end