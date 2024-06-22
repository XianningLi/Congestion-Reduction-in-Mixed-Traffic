[a1, a2] = ndgrid(velocities);
vels = [params.veh.initialVals(4)*ones(numel(a1),1) a1(:) a2(:)];
n = size(vels,2)-1;
indx = round(length(arcLengthAll)/n);
accelerations = [(vels(:,2).^2-vels(:,1).^2)./(2*arcLengthAll(indx)) (vels(:,3).^2-vels(:,2).^2)./(2*(arcLengthAll(end)-arcLengthAll(indx)))];

velocitiesAll = zeros(size(vels,1), numel(arcLengthAll));
accelerationAll = zeros(size(vels,1), numel(arcLengthAll));
timeTakenAll = zeros(size(vels,1), numel(arcLengthAll));
for i = 1:size(vels, 1)
    time0 = 0;
    velocitiesAll(i,:) = [ sqrt(vels(i,1)^2+2*accelerations(i,1)*arcLengthAll(1:indx))  sqrt(vels(i,2)^2+2*accelerations(i,2)*(arcLengthAll(indx+1:end)-arcLengthAll(indx)))];
    accelerationAll(i,:) = [ accelerations(i,1)*ones(1,length(arcLengthAll(1:indx))) accelerations(i,2)*ones(1, length(arcLengthAll(indx+1:end)) )];

    if accelerations(i,1) == 0
        timeTakenAll(i,1:indx) = time0 + arcLengthAll(1:indx)./velocitiesAll(i,1:indx);
        time0 = timeTakenAll(i, indx);
    else
        timeTakenAll(i,1:indx) = time0 + (velocitiesAll(i,1:indx)-vels(i,1))/accelerations(i,1);
        time0 = timeTakenAll(i, indx);
    end

    if accelerations(i,2) == 0
        timeTakenAll(i,indx+1:end) = time0 + (arcLengthAll(indx+1:end)-arcLengthAll(indx))./velocitiesAll(i,indx+1:end);
        time0 = timeTakenAll(i, end);
    else
        timeTakenAll(i,indx+1:end) = time0 + (velocitiesAll(i,indx+1:end)-vels(i,2))/accelerations(i,2);
        time0 = timeTakenAll(i, end);
    end
end
j = 9;
params.xyPlan.x = [bicycleStatesXYTheta(1:3, :); velocitiesAll(j,:)];
params.xyPlan.u = steeringAll;
params.opti.N = numel(timeTakenAll(j,:));
params.opti.dt =  timeTakenAll(j,:);
params.veh.initialVals(4) = v0;
params.veh.accel = accelerationAll(j,:);
[z, susForce, totalCost, searchTimeZ] = genMotionPlanZLattice(params);

%% visualization

x = [bicycleStatesXYTheta; velocitiesAll(j,:); z];
u = [steeringAll(1:end-1); accelerationAll(j,1:end-1); susForce];

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
plot(u(3,:), 'linewidth', 1)
hold on
plot(u(4,:), 'linewidth', 1)
hold on
plot(u(5,:), 'linewidth', 1)
hold on
plot(u(6,:), 'linewidth', 1)
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
