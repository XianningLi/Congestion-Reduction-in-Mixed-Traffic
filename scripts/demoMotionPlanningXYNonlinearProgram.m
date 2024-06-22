clear all
close all
clc

%% Initialization
fileName = 'potBumpTestCase16';
params = loadParams(strcat(fileName,'.json')); % run the parameter file

%% Motion planning
tic
[x,u] = genMotionPlanXY(params);
toc
%% compute estimated road
Zest = params.road.RBFBasis*params.road.alpha;  % compute estimated road profile
Zest = reshape(Zest, round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));
params.road.Zest = Zest;

save(strcat('C:\git\research-2023-motion-planning\matFiles\', fileName, 'XYPlan' ,'.mat'))
%% Visualization

figure(1)
mesh(params.road.X, params.road.Y, Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
hold on
plot(x(1,:), x(2,:), 'o-', 'linewidth', 0.5)
xlabel('x')
ylabel('y')
zlabel('z')
% xlim([0 params.road.Nx])
hold off

coords = getCornerCoords(params);

figure(2)
plot(0:params.opti.N, coords.road.rFL(x))
hold on
plot(0:params.opti.N, coords.road.rFR(x))
hold on
plot(0:params.opti.N, coords.road.rRL(x))
hold on
plot(0:params.opti.N, coords.road.rRR(x))
legend('rFL', 'rFR', 'rRL', 'rRR')
xlabel('time steps')


figure(3) % plot control inputs
plot(rad2deg(u(1,:)), 'linewidth', 1)
legend('steering')
xlabel('time steps')

figure(5) % plot xy data
hold on
plot(x(1,:), x(2,:), '-.', 'linewidth', 1)
axis equal
legend('x-y trajectory')
xlabel('time steps')



%%
plotTraj(x, u, params, 0, strcat(fileName,'Demo'))
% generateGifNew(0, 'demo', x, params)