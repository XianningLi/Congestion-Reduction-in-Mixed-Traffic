function [x,u] = genMotionPlanXY(params)
% Main motion planning function that creates XYZ motion plans


import casadi.*

N = params.opti.N; % number of steps

opti = casadi.Opti(); % creating a casadi optimization object
X = opti.variable(3,N+1); % added a state v_dot = u/m, u = force in acc pedal, m = mass of vehicle = 1
U = opti.variable(1,N); % speed, steering (curvature), and 4 active suspension forces

% constraints
% Initial contidion
initialVals = params.veh.initialVals;
opti.subject_to(X(:,1) == initialVals(1:3))

% dynamic constraints
f = @(x,u) [params.opti.vref*cos(x(3)); 
            params.opti.vref*sin(x(3)); 
            u(1)*params.opti.vref;]; %x(1) = x, x(2) = y, x(3) = theta, x(4) = V, x(5) = z1, x(6) = z2, u(1) = k, u(2) = acc., u(3) = fa
dt = params.opti.dt;

% objective 
total_cost = local_compute_cost(X, U, params);
opti.minimize(total_cost);

for k = 1:N
    x_next = X(:,k) + dt*f(X(:,k), U(:,k));
    opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% control limit
opti.subject_to(params.ctrlLims.uLim(1,1)<=U<=params.ctrlLims.uLim(1,2)) % equivalent to 30 deg

% state limit
opti.subject_to(params.stateLims.xLim(1,1)<=X(1,:)) % road x limit
opti.subject_to(params.stateLims.xLim(2,1)<=X(2,:)<=params.stateLims.xLim(2,2)) % road y limit

opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yFL(X)<=params.stateLims.xLim(2,2))
opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yFR(X)<=params.stateLims.xLim(2,2))
opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yRL(X)<=params.stateLims.xLim(2,2))
opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yRR(X)<=params.stateLims.xLim(2,2))

% ---- solve NLP              ------

opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

x = sol.value(X);
u = sol.value(U);

end

%% Local functions
function total_cost = local_compute_cost(X, U, params)
weights_cost_rxy = [1500 1500 1500 1500];
% weights_cost_diff = [20 20 20];
cost_rxy = [
            getCornerCoords(params).road.rFL(X)*getCornerCoords(params).road.rFL(X)'
            getCornerCoords(params).road.rFR(X)*getCornerCoords(params).road.rFR(X)'
            getCornerCoords(params).road.rRL(X)*getCornerCoords(params).road.rRL(X)'
            getCornerCoords(params).road.rRR(X)*getCornerCoords(params).road.rRR(X)'];

cost_lat      =  [X(3,:)*X(3,:)'
                  (X(2,:)-params.opti.XY(1))*(X(2,:)-params.opti.XY(1))'
                  U(1,:)*U(1,:)'];

% cost_diff = [diff(U(1,:))*diff(U(1,:))'
%              diff(X(3,:))*diff(X(3,:))'
%              diff(diff(X(2,:)))*diff(diff(X(2,:)))'];

total_cost    = params.opti.weights.cost_lat*cost_lat ...
                + weights_cost_rxy*cost_rxy;

end

