
%% generate velocity, acceleration, time profiles
velocities = [params.veh.initialVals(4)+5 params.veh.initialVals(4)+3 params.veh.initialVals(4) params.veh.initialVals(4)-3 params.veh.initialVals(4)-5];
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


%% perform z planning
z = zeros(6, size(timeTakenAll,2), size(timeTakenAll,1));
susForce = zeros(4, size(timeTakenAll,2)-1, size(timeTakenAll,1));
totalCostZ = nan(1, size(timeTakenAll,1));
searchTimeZ = zeros(1, size(timeTakenAll,1));

for j = 1:size(timeTakenAll,1)
    params.xyPlan.x = [bicycleStatesXYTheta(1:3, :); velocitiesAll(j,:)];
    params.xyPlan.u = steeringAll;
    params.opti.N = numel(timeTakenAll(j,:));
    params.opti.dt =  timeTakenAll(j,:);
    params.veh.initialVals(4) = v0;
    params.veh.accel = accelerationsAll(j,:);
    try
        [z(:, :, j), susForce(:, :, j), totalCostZ(j), searchTimeZ(j)] = genMotionPlanZLattice(params);
    catch
        warning("verticle optimization failed for the given (t, v, a) profile. Trying next profile...")
    end
end

% select best Z
[~, minIndx] = min(totalCostZ);
%% visualization
zBest = z(:, :, minIndx);
susForceBest = susForce(:, :, minIndx);
timeTakenBest = timeTakenAll(minIndx, :);

x = [bicycleStatesXYTheta; velocitiesAll(minIndx, :); zBest];
u = [steeringAll(1:end-1); accelerationsAll(minIndx, 1:end-1); susForceBest];

figure
mesh(params.road.X, params.road.Y, params.road.Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
hold on
plot3(x(1,:), x(2,:), x(5,:), 'ro-', 'linewidth', 0.5)
xlabel('x')
ylabel('y')
zlabel('z')
hold on
circX = params.xyPlan.obsX + params.xyPlan.obsRadi*cos(0:0.1:2*pi);
circY = params.xyPlan.obsY + params.xyPlan.obsRadi*sin(0:0.1:2*pi);
circZ = 0.06*ones(1, numel(circY));
fill3(circX, circY, circZ, 'm')
hold off

coords = getCornerCoords(params);
forces = getCornerForce(params);

figure
subplot(2,2,1)
plot(coords.XYZ.zFL(x))
hold on
plot(coords.XYZ.zFR(x))
hold on
plot(coords.XYZ.zRL(x))
hold on
plot(coords.XYZ.zRR(x))
legend('zFL', 'zFR', 'zRL', 'zRR')
xlabel('time steps')
subplot(2,2,2)
plot(coords.road.rFL(x))
hold on
plot(coords.road.rFR(x))
hold on
plot(coords.road.rRL(x))
hold on
plot(coords.road.rRR(x))
legend('rFL', 'rFR', 'rRL', 'rRR')
xlabel('time steps')
subplot(2,2,3)
plot(coords.XYZ.zFL(x)-coords.road.rFL(x))
hold on
plot(coords.XYZ.zFR(x)-coords.road.rFR(x))
hold on
plot(coords.XYZ.zRL(x)-coords.road.rRL(x))
hold on
plot(coords.XYZ.zRR(x)-coords.road.rRR(x))
legend('zFL-rFL', 'zFR-rFR', 'zRL-rRL', 'zRR-rRR')
xlabel('time steps')
subplot(2,2,4)
hold on
plot(timeTakenBest(1:end-1), u(3,:), 'linewidth', 1)
hold on
plot(timeTakenBest(1:end-1), u(4,:), 'linewidth', 1)
hold on
plot(timeTakenBest(1:end-1), u(5,:), 'linewidth', 1)
hold on
plot(timeTakenBest(1:end-1), u(6,:), 'linewidth', 1)
legend('faFL', 'faFR', 'faRL', 'faRR')
xlabel('time steps')

figure % plot control inputs
subplot(2,2,1)
hold on
plot(u(1,:), 'linewidth', 1)
legend('steering')
xlabel('time steps')
subplot(2,2,2)
plot(u(2,:), 'linewidth', 1)
legend('acceleration')
xlabel('time steps')
subplot(2,2,3)
hold on
plot(u(3,:), 'linewidth', 1)
hold on
plot(u(4,:), 'linewidth', 1)
hold on
plot(u(5,:), 'linewidth', 1)
hold on
plot(u(6,:), 'linewidth', 1)
legend('faFL', 'faFR', 'faRL', 'faRR')
xlabel('time steps')
subplot(2,2,4)
hold on
plot(forces.fFL(x(:,1:end-1),u), 'linewidth', 1)
hold on
plot(forces.fFR(x(:,1:end-1),u), 'linewidth', 1)
hold on
plot(forces.fRL(x(:,1:end-1),u), 'linewidth', 1)
hold on
plot(forces.fRR(x(:,1:end-1),u), 'linewidth', 1)
legend('fFL', 'fFR', 'fRL', 'fRR')
xlabel('time steps')

figure % plot z data
subplot(2,2,1)
hold on
plot(x(5,:), 'o-', 'linewidth', 0.5)
legend('heave')
xlabel('time steps')
subplot(2,2,2)
hold on
plot(x(7,:), 'o-', 'linewidth', 0.5)
legend('pitch')
xlabel('time steps')
subplot(2,2,3)
hold on
plot(x(9,:), 'o-', 'linewidth', 0.5)
legend('roll')
xlabel('time steps')
subplot(2,2,4)
hold on
plot(params.road.RoadData.fun1(x), 'o-', 'linewidth', 0.5)
legend('r(x,y)')
xlabel('time steps')

figure % plot xy data
hold on
plot(x(1,:), x(2,:), '-.', 'linewidth', 1)
axis equal
legend('x-y trajectory')
xlabel('time steps')

figure % plot velocity
hold on
plot(x(4,:), '-.', 'linewidth', 1)
legend('velocity')
xlabel('time steps')
