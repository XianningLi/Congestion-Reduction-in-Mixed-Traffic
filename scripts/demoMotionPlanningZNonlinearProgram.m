clear all
close all
clc

%% Initialization
fileName = 'fennLanesEB_X180m280m_Y1m3p5m';
% fileName = 'potBumpTestCase18';
params = loadParams(strcat(fileName,'.json')); % run the parameter file
params.xyPlan = load(strcat(fileName, 'XYPlan' ,'.mat'));
%% Motion planning
tic
[x,u] = genMotionPlanZ(params);
toc
%% compute estimated road
Zest = params.road.RBFBasis*params.road.alpha;  % compute estimated road profile
Zest = reshape(Zest, round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));
params.road.Zest = Zest;

%% get the arc lengths

s0 = 0;
sAll = [s0];
for i=1:numel(x(1,:))
    s = s0 + params.opti.vref*params.opti.dt;
    sAll = [sAll s];
    s0 = s;
end

xyTempoX = interp1(params.xyPlan.s, params.xyPlan.x(1,:), sAll);
xyTempoY = interp1(params.xyPlan.s, params.xyPlan.x(2,:), sAll);
xyTempoTh = interp1(params.xyPlan.s, params.xyPlan.x(3,:), sAll);

uTempo = interp1(params.xyPlan.s, params.xyPlan.u, sAll);

plot(xyTempoX, xyTempoY)
x = [xyTempoX(1:end-1); xyTempoY(1:end-1); xyTempoTh(1:end-1); x];
u = [uTempo(1:end-2);u];

% x = [params.xyPlan.x;x];
% u = [params.xyPlan.u;u];

% save(strcat('C:\git\research-2023-motion-planning\matFiles\', fileName, 'ZPlan' ,'.mat'))

%% Visualization

figure(1)
mesh(params.road.X, params.road.Y, params.road.Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
hold on
plot3(x(1,:), x(2,:), x(5,:), 'o-', 'linewidth', 0.5)
xlabel('x')
ylabel('y')
zlabel('z')
hold on
circX = 48+0.3*cos(0:0.1:2*pi);
circY = 1+0.3*sin(0:0.1:2*pi);
circZ = 0.06*ones(1, numel(circY));
fill3(circX, circY, circZ, 'm')
hold off

coords = getCornerCoords(params);
forces = getCornerForce(params);

figure(2)
subplot(2,2,1)
plot(0:params.opti.N, coords.XYZ.zFL(x))
hold on
plot(0:params.opti.N, coords.XYZ.zFR(x))
hold on
plot(0:params.opti.N, coords.XYZ.zRL(x))
hold on
plot(0:params.opti.N, coords.XYZ.zRR(x))
legend('zFL', 'zFR', 'zRL', 'zRR')
xlabel('time steps')
subplot(2,2,2)
plot(0:params.opti.N, coords.road.rFL(x))
hold on
plot(0:params.opti.N, coords.road.rFR(x))
hold on
plot(0:params.opti.N, coords.road.rRL(x))
hold on
plot(0:params.opti.N, coords.road.rRR(x))
legend('rFL', 'rFR', 'rRL', 'rRR')
xlabel('time steps')
subplot(2,2,3)
plot(0:params.opti.N, coords.XYZ.zFL(x)-coords.road.rFL(x))
hold on
plot(0:params.opti.N, coords.XYZ.zFR(x)-coords.road.rFR(x))
hold on
plot(0:params.opti.N, coords.XYZ.zRL(x)-coords.road.rRL(x))
hold on
plot(0:params.opti.N, coords.XYZ.zRR(x)-coords.road.rRR(x))
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

figure(3) % plot control inputs
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

figure(4) % plot z data
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

figure(5) % plot xy data
hold on
plot(x(1,:), x(2,:), '-.', 'linewidth', 1)
axis equal
legend('x-y trajectory')
xlabel('time steps')

figure(6) % plot velocity
hold on
plot(x(4,:), '-.', 'linewidth', 1)
legend('velocity')
xlabel('time steps')

% [Xm, Ym] = meshgrid(x(1,:), x(2,:));
% % Zest = RBFBasisLongLat(params.road.decayRate, params.road.X(:), params.road.Y(:), params.road.xxb(:)', params.road.yyb(:)')*params.road.alpha;
% Zest = RBFBasisLongLat(params.road.decayRate, Xm(:), Ym(:), params.road.xxb(:)', params.road.yyb(:)')*params.road.alpha;
% figure(7)
% % mesh(params.road.X, params.road.Y, reshape(Zest, size(params.road.X)))
% mesh(Xm, Ym, reshape(Zest, size(Xm)))
% daspect([1 1 1])

%%
plotTraj(x, u, params, 0, strcat(fileName,'Demo'))
% generateGifNew(0, 'demo', x, params)