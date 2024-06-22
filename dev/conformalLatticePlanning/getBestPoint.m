function [bestPoint, bestP] = getBestPoint(params, updatedP, bestPrev, targets)
% this fuction outputs the best path give a set of sprial parameters that
% connects the previous point bestPrev to the targets


% initialize the cost vector
costVec = zeros(1,size(updatedP,2));

for i = 1:size(updatedP,2)
    % get the bicycle states for given spiral parameters
    bicycleStates = computeKinematics(updatedP(:,i), 0:0.01:updatedP(end,i), bestPrev, params);

    % get the cost vector for road disturbances at four corners of the vehicle
    cost_rxy = [
        getCornerCoords(params).road.rFL(bicycleStates)*getCornerCoords(params).road.rFL(bicycleStates)'
        getCornerCoords(params).road.rFR(bicycleStates)*getCornerCoords(params).road.rFR(bicycleStates)'
        getCornerCoords(params).road.rRL(bicycleStates)*getCornerCoords(params).road.rRL(bicycleStates)'
        getCornerCoords(params).road.rRR(bicycleStates)*getCornerCoords(params).road.rRR(bicycleStates)'];
    
    % get cost for deviation from the lane center 
    cost_laneCenter = (bicycleStates(2,:)-params.xyPlan.laneCenter)*(bicycleStates(2,:)-params.xyPlan.laneCenter)';

    % barrier for obstacle from CG
    h = (bicycleStates(1,:)-params.xyPlan.obsX).^2 + (bicycleStates(2,:)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2;  

    % barrier for obstacle from four corners
    hFL = (getCornerCoords(params).XYZ.xFL(bicycleStates)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yFL(bicycleStates)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2; 
    hFR = (getCornerCoords(params).XYZ.xFR(bicycleStates)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yFR(bicycleStates)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2; 
    hRL = (getCornerCoords(params).XYZ.xRL(bicycleStates)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yRL(bicycleStates)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2; 
    hRR = (getCornerCoords(params).XYZ.xRR(bicycleStates)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yRR(bicycleStates)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2; 
    
    % get the total cost (at this point, the weights for h, hFL, hFR, hRL, hRR are fixed here)
%     costVec(i) =  params.xyPlan.weigths_laneCenter*cost_laneCenter + params.xyPlan.weights_cost_rxy*cost_rxy...
%         + (1e10/sum(h) + 1e10/sum(hFL) + 1e10/sum(hFR) + 1e10/sum(hRL) + 1e10/sum(hRR));
   costVec(i) =  params.xyPlan.weigths_laneCenter*cost_laneCenter + params.xyPlan.weights_cost_rxy*cost_rxy...
        + params.xyPlan.weights_cost_obs*( exp(-hFL)*exp(-hFL)' + exp(-hFR)*exp(-hFR)' + exp(-hRL)*exp(-hRL)' + exp(-hRR)*exp(-hRR)' );
end

% get the minimum
[~,I] = min(costVec);

% select the best
bestP = updatedP(:,I);
bestPoint = targets(:,I);
end