function bestZPlanIndx = getBestZPlan(params,totalCostZ,velocitiesAll)

velNorm = sum((velocitiesAll-params.opti.vref).^2,2);
zCosts = params.xyPlan.velWeigth*velNorm + params.xyPlan.zCostWeigth*totalCostZ';
[~, bestZPlanIndx] = min(zCosts);

end