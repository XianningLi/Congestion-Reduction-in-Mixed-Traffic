clear all
close all
clc
fileName = 'fennLanesEB_X180m230m_Y1m3p5mZPlan';
Data = load(strcat(fileName,'.mat'));
params = Data.params;
f = @(x,u) [params.opti.vref*cos(x(3)); 
            params.opti.vref*sin(x(3)); 
            u(1)*params.opti.vref; 
            u(2);
            x(6); 
            (1/params.veh.m)*(getCornerForce(params).fFL(x, u)+getCornerForce(params).fFR(x, u)+getCornerForce(params).fRL(x, u)+getCornerForce(params).fRR(x, u));
            x(8);
            (1/params.veh.Jb)*(-params.veh.l1*getCornerForce(params).fFL(x, u)-params.veh.l1*getCornerForce(params).fFR(x, u)+params.veh.l2*getCornerForce(params).fRL(x, u)+params.veh.l2*getCornerForce(params).fRR(x, u));
            x(10);
            (1/params.veh.Jg)*(params.veh.w*getCornerForce(params).fFL(x, u)-params.veh.w*getCornerForce(params).fFR(x, u)+params.veh.w*getCornerForce(params).fRL(x, u)-params.veh.w*getCornerForce(params).fRR(x, u));]; 

X0 = params.veh.initialVals;
dt = params.opti.dt;
x_all = [X0];
for k = 1:params.opti.N
    U = Data.u(:,k);
    x_next = X0 + dt*f(X0, [U(1) U(2) 0 0 0 0]');
    x_all = [x_all x_next];
    X0 = x_next;
end

%% Visualization
x = x_all;
u = Data.u;
u(3:6, :) = 0;

Zest = params.road.RBFBasis*params.road.alpha;  % compute estimated road profile
Zest = reshape(Zest, round(params.road.Ny/params.road.gridSize), round(params.road.Nx/params.road.gridSize));
params.road.Zest = Zest;

figure
mesh(params.road.X, params.road.Y, Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
hold on
plot3(x(1,:), x(2,:), x(5,:), 'o-', 'linewidth', 0.5)
xlabel('x')
ylabel('y')
zlabel('z')
hold off

coords = getCornerCoords(params);
forces = getCornerForce(params);

figure
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

% [Xm, Ym] = meshgrid(x(1,:), x(2,:));
% % Zest = RBFBasisLongLat(params.road.decayRate, params.road.X(:), params.road.Y(:), params.road.xxb(:)', params.road.yyb(:)')*params.road.alpha;
% Zest = RBFBasisLongLat(params.road.decayRate, Xm(:), Ym(:), params.road.xxb(:)', params.road.yyb(:)')*params.road.alpha;
% figure(7)
% % mesh(params.road.X, params.road.Y, reshape(Zest, size(params.road.X)))
% mesh(Xm, Ym, reshape(Zest, size(Xm)))
% daspect([1 1 1])


%%
plotTraj(x, u, params, 0, strcat('plan', fileName,'DemoNoSus'))
% generateGifNew(0, 'demo', x, params)



