clear all
close all
clc
%% Initialization
% fileName = 'potBumpTestCase16';
fileName = 'fennLanesEB_X180m280m_Y1m3p5m';
% fileName = 'fennLanesEB_X2900m3000m_Y1m3p5m';
params = loadParams(strcat(fileName,'.json')); % run the parameter file
%% Motion planning
tic
[x, u, computeTime] = genMotionPlan(params);
toc
%% compute estimated road
Zest = params.road.RBFBasis*params.road.alpha;  % compute estimated road profile
Zest = reshape(Zest, round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));
params.road.Zest = Zest;
save(strcat('D:\Sayan Github\research-2023-motion-planning\matFiles\', fileName, 'PlanNoObstacle' ,'.mat'))
%% Visualization

figure
mesh(params.road.X, params.road.Y, params.road.Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
hold on
plot3(x(1,:), x(2,:), 0.1+x(5,:), 'r', 'linewidth', 3)
xlabel('$x~(m)$', 'interpreter', 'latex')
ylabel('$y~(m)$', 'interpreter', 'latex')
zlabel('$z~(m)$', 'interpreter', 'latex')
hold on
circX = params.xyPlan.obsX + params.xyPlan.obsRadi*cos(0:0.1:2*pi);
circY = params.xyPlan.obsY + params.xyPlan.obsRadi*sin(0:0.1:2*pi);
circZ = 0.5*ones(1, numel(circY));
fill3(circX, circY, circZ, 'm')
hold off
box off
set(gca, 'fontsize', 25)
xlim([0 100])

coords = getCornerCoords(params);
forces = getCornerForce(params);

timeTaken = 0:params.opti.dt:params.opti.N*params.opti.dt;

figure
plot(timeTaken, coords.XYZ.zFL(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.XYZ.zFR(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.XYZ.zRL(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.XYZ.zRR(x), 'linewidth', 2)
h = legend('$z_1$', '$z_2$', '$z_3$', '$z_4$');
set(h,'interpreter', 'latex')
legend boxoff
xlabel('Time $(s)$', 'interpreter', 'latex')
ylabel('Vertical displacement $(m)$', 'interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

figure
plot(timeTaken, coords.road.rFL(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.road.rFR(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.road.rRL(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.road.rRR(x), 'linewidth', 2)
h = legend('$r_1$', '$r_2$', '$r_3$', '$r_4$');
set(h,'interpreter', 'latex')
legend boxoff
xlabel('Time $(s)$', 'interpreter', 'latex')
ylabel('Road height $(m)$', 'interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

figure
plot(timeTaken, coords.XYZ.zFL(x)-coords.road.rFL(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.XYZ.zFR(x)-coords.road.rFR(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.XYZ.zRL(x)-coords.road.rRL(x), 'linewidth', 2)
hold on
plot(timeTaken, coords.XYZ.zRR(x)-coords.road.rRR(x), 'linewidth', 2)
h = legend('$z_{1}-r_{1}$', '$z_{2}-r_{2}$', '$z_{3}-r_{3}$', '$z_{4}-r_{4}$');
set(h,'interpreter', 'latex')
legend boxoff
xlabel('Time $(s)$', 'interpreter', 'latex')
ylabel('Suspension travel $(m)$', 'interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

figure
hold on
plot(timeTaken(1:end-1), u(3,:), 'linewidth', 2)
hold on
plot(timeTaken(1:end-1), u(4,:), 'linewidth', 2)
hold on
plot(timeTaken(1:end-1), u(5,:), 'linewidth', 2)
hold on
plot(timeTaken(1:end-1), u(6,:), 'linewidth', 2)
h = legend('$f_{a1}$', '$f_{a2}$', '$f_{a3}$', '$f_{a4}$');
set(h,'interpreter', 'latex')
legend boxoff
xlabel('Time $(s)$', 'interpreter', 'latex')
ylabel('Suspension force $(N)$', 'interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

figure
plot(timeTaken(1:end-1), u(1,:), 'linewidth', 2)
h = legend('steering');
set(h,'interpreter', 'latex')
legend boxoff
xlabel('Time $(s)$','interpreter', 'latex')
ylabel('Steering (rad)', 'interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

figure
plot(timeTaken(1:end-1), u(2,:), 'linewidth', 2)
xlabel('Time $(s)$','interpreter', 'latex')
ylabel('Acceleration $(m/s^2)$','interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

figure % plot xy data
hold on
plot(x(1,:), x(2,:), 'linewidth', 2)
axis equal
xlabel('$x (m)$','interpreter', 'latex')
ylabel('$y (m)$','interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

figure % plot velocity
hold on
plot(timeTaken, x(4,:), 'linewidth', 2)
xlabel('Time $(s)$','interpreter', 'latex')
ylabel('Velocity $(m/s)$','interpreter', 'latex')
set(gca, 'fontsize', 25)
box off
xlim([0 6])

%%
plotTraj(x, u, params, 0, strcat(fileName,'Demo'))
% generateGifNew(0, 'demo', x, params)