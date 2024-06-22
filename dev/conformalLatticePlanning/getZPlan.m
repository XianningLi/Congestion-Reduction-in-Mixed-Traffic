function [minIndxZ, z, susForce, searchTimeZ, totalCostZ] = getZPlan(params)
%% perform z planning
z = zeros(6, size(params.xyPlan.timeTakenAll,2), size(params.xyPlan.timeTakenAll,1));
susForce = zeros(4, size(params.xyPlan.timeTakenAll,2)-1, size(params.xyPlan.timeTakenAll,1));
totalCostZ = nan(1, size(params.xyPlan.timeTakenAll,1));
searchTimeZ = zeros(1, size(params.xyPlan.timeTakenAll,1));
for j = 1:size(params.xyPlan.timeTakenAll,1)
    params.xyPlan.x = [params.xyPlan.bicycleStatesXYTheta(1:3, :); params.xyPlan.velocitiesAll(j,:)];
    params.xyPlan.u = params.xyPlan.steeringAll;
    params.opti.N = numel(params.xyPlan.timeTakenAll(j,:));
    params.opti.dt =  params.xyPlan.timeTakenAll(j,:);
    % params.veh.initialVals(4) = params.veh.initialVals(4);
    params.veh.accel = params.xyPlan.accelerationsAll(j,:);
    try
        [z(:, :, j), susForce(:, :, j), totalCostZ(j), searchTimeZ(j)] = genMotionPlanZLattice(params);
        searchTimeZ(j)
    catch
        warning("verticle optimization failed for the given (t, v, a) profile. Trying next profile...")
    end
end
% select best Z
% [~, minIndxZ] = min(totalCostZ);
minIndxZ = getBestZPlan(params,totalCostZ,velocitiesAll);

end