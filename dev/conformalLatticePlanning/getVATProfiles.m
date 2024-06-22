function [timeTakenAll, velocitiesAll, accelerationsAll] = getVATProfiles(arcLengthAll, params)
%% generate velocity, acceleration, and time profiles
velocities = [params.veh.initialVals(4)+6 params.veh.initialVals(4)+5 params.veh.initialVals(4)+3 params.veh.initialVals(4) params.veh.initialVals(4)-3 params.veh.initialVals(4)-5 params.veh.initialVals(4)-6];
[a1, a2] = ndgrid(velocities);
vels = [params.veh.initialVals(4)*ones(numel(a1),1) a1(:) a2(:)];
n = size(vels,2)-1;
indx = round(length(arcLengthAll)/n);
velocitiesAll = zeros(size(vels,1), numel(arcLengthAll));
accelerationsAll = zeros(size(vels,1), numel(arcLengthAll));
timeTakenAll = zeros(size(vels,1), numel(arcLengthAll));
for i = 1:size(vels, 1)
    time0 = 0;
    paramsVelAccel1.s0 = arcLengthAll(1);
    paramsVelAccel1.s1 = arcLengthAll(indx+1);
    paramsVelAccel1.v0 = vels(i,1);
    paramsVelAccel1.v1 = vels(i,2);
    paramsVelAccel1.a0 = 0;
    paramsVelAccel1.a1 = 0;
    paramsVelAccel1.s = arcLengthAll(1:indx);

    paramsVelAccel2.s0 = arcLengthAll(indx+1);
    paramsVelAccel2.s1 = arcLengthAll(end);
    paramsVelAccel2.v0 = vels(i,2);
    paramsVelAccel2.v1 = vels(i,3);
    paramsVelAccel2.a0 = 0;
    paramsVelAccel2.a1 = 0;
    paramsVelAccel2.s = arcLengthAll(indx+1:end);

    [velocitiesAll(i,1:indx), accelerationsAll(i,1:indx)] = getVelAccelSpriral(paramsVelAccel1);
    [velocitiesAll(i,indx+1:end), accelerationsAll(i,indx+1:end)] = getVelAccelSpriral(paramsVelAccel2);

    del_t = gradient(paramsVelAccel1.s)./velocitiesAll(i,1:indx);
    timeTakenAll(i, 1:indx) = time0+cumsum(del_t);
    time0 = timeTakenAll(i, indx);

    del_t = gradient(paramsVelAccel2.s)./velocitiesAll(i,indx+1:end);
    timeTakenAll(i, indx+1:end) = time0+cumsum(del_t);
    time0 = timeTakenAll(i, indx+1:end);
end
end