function [x, u, computeTime] = genMotionPlan(params)
% Main motion planning function that creates XYZ motion plans

import casadi.*

N = params.opti.N; % number of steps

opti = casadi.Opti();      % creating a casadi optimization object
X = opti.variable(10,N+1); % added a state v_dot = u/m, u = force in acc pedal, m = mass of vehicle = 1
U = opti.variable(6,N);    % speed, steering (curvature), and 4 active suspension forces

% constraints
% Initial contidion
initialVals = params.veh.initialVals;
opti.subject_to(X(:,1) == initialVals)
% opti.subject_to(U(:,2) == 1)

% opti.subject_to(X(1,1) == initialVals(1));   % x
% opti.subject_to(X(2,1) == initialVals(2));   % y
% opti.subject_to(X(3,1) == initialVals(3));   % theta
% opti.subject_to(X(4,1) == initialVals(4));   % v
% opti.subject_to(X(5,1) == initialVals(5));   % z1 (heave)
% opti.subject_to(X(6,1) == initialVals(6));   % z2 (heave dot)
% opti.subject_to(X(7,1) == initialVals(7));   % beta1 (pitch)
% opti.subject_to(X(8,1) == initialVals(8));   % beta2 (pitch dot)
% opti.subject_to(X(9,1) == initialVals(9));   % gamma1 (roll)
% opti.subject_to(X(10,1) == initialVals(10)); % gamma2 (roll dot)

% dynamic constraints
f = @(x,u) [x(4)*cos(x(3)); 
            x(4)*sin(x(3)); 
            u(1)*x(4); 
            u(2);
            x(6); 
            (1/params.veh.m)*(getCornerForce(params).fFL(x, u)+getCornerForce(params).fFR(x, u)+getCornerForce(params).fRL(x, u)+getCornerForce(params).fRR(x, u));
            x(8);
            (1/params.veh.Jb)*(-params.veh.l1*getCornerForce(params).fFL(x, u)-params.veh.l1*getCornerForce(params).fFR(x, u)+params.veh.l2*getCornerForce(params).fRL(x, u)+params.veh.l2*getCornerForce(params).fRR(x, u));
            x(10);
            (1/params.veh.Jg)*(params.veh.w*getCornerForce(params).fFL(x, u)-params.veh.w*getCornerForce(params).fFR(x, u)+params.veh.w*getCornerForce(params).fRL(x, u)-params.veh.w*getCornerForce(params).fRR(x, u));]; %x(1) = x, x(2) = y, x(3) = theta, x(4) = V, x(5) = z1, x(6) = z2, u(1) = k, u(2) = acc., u(3) = fa
dt = params.opti.dt;

% objective 
total_cost = local_compute_cost(X, U, params);
opti.minimize(total_cost);

for k = 1:N
    % Runge-Kutta 4 integration
    %     k1 = f(X(:,k),         U(:,k));
    %     k2 = f(X(:,k)+dt/2*k1, U(:,k));
    %     k3 = f(X(:,k)+dt/2*k2, U(:,k));
    %     k4 = f(X(:,k)+dt*k3,   U(:,k));
    %     x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4);
    x_next = X(:,k) + dt*f(X(:,k), U(:,k));
    opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% control limit
opti.subject_to(params.ctrlLims.uLim(:,1)<=U<=params.ctrlLims.uLim(:,2)) % equivalent to 30 deg

% Avoid twist force
opti.subject_to([1 -1 -1 1]*U(3:6,:)==0)

% state limit
opti.subject_to(params.stateLims.xLim(1,1)<=X(1,:)) % road x limit
opti.subject_to(params.stateLims.xLim(2,1)<=X(2,:)<=params.stateLims.xLim(2,2)) % road y limit
opti.subject_to(params.stateLims.xLim(4,1)<=X(4,:)<=params.stateLims.xLim(4,2))

opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yFL(X)<=params.stateLims.xLim(2,2))
opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yFR(X)<=params.stateLims.xLim(2,2))
opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yRL(X)<=params.stateLims.xLim(2,2))
opti.subject_to(params.stateLims.xLim(2,1)<=getCornerCoords(params).XYZ.yRR(X)<=params.stateLims.xLim(2,2))

% opti.subject_to(params.stateLims.xLim(1,1)<=getCornerCoords(params).XYZ.xFL(X)<=params.stateLims.xLim(1,2))
% opti.subject_to(params.stateLims.xLim(1,1)<=getCornerCoords(params).XYZ.xFR(X)<=params.stateLims.xLim(1,2))
% opti.subject_to(params.stateLims.xLim(1,1)<=getCornerCoords(params).XYZ.xRL(X)<=params.stateLims.xLim(1,2))
% opti.subject_to(params.stateLims.xLim(1,1)<=getCornerCoords(params).XYZ.xRR(X)<=params.stateLims.xLim(1,2))

% set limit on z displacement for four corners
opti.subject_to( params.veh.susTravLim(1,1) <= getCornerCoords(params).XYZ.zFL(X) - getCornerCoords(params).road.rFL(X) <= params.veh.susTravLim(1,2))
opti.subject_to( params.veh.susTravLim(2,1) <= getCornerCoords(params).XYZ.zFR(X) - getCornerCoords(params).road.rFR(X) <= params.veh.susTravLim(2,2))
opti.subject_to( params.veh.susTravLim(3,1) <= getCornerCoords(params).XYZ.zRL(X) - getCornerCoords(params).road.rRL(X) <= params.veh.susTravLim(3,2))
opti.subject_to( params.veh.susTravLim(4,1) <= getCornerCoords(params).XYZ.zRR(X) - getCornerCoords(params).road.rRR(X) <= params.veh.susTravLim(4,2))



% obstacle
% opti.subject_to((X(1,:)-params.xyPlan.obsX).^2 + (X(2,:)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2 >= 0);
% opti.subject_to((getCornerCoords(params).XYZ.xFL(X)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yFL(X)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2 >= 0); 
% opti.subject_to((getCornerCoords(params).XYZ.xFR(X)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yFR(X)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2 >= 0); 
% opti.subject_to((getCornerCoords(params).XYZ.xRL(X)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yRL(X)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2 >= 0); 
% opti.subject_to((getCornerCoords(params).XYZ.xRR(X)-params.xyPlan.obsX).^2 + (getCornerCoords(params).XYZ.yRR(X)-params.xyPlan.obsY).^2 - params.xyPlan.obsRadi^2 >= 0); 

% ---- solve NLP              ------
% Create an options structure for IPOPT
opts = struct('ipopt', struct('print_level', 0), 'print_time', 0);

% Apply the solver with options to suppress output
opti.solver('ipopt', opts);

% opti.solver('ipopt'); % set numerical backend
tic
sol = opti.solve();   % actual solve
computeTime = toc;
x = sol.value(X);
u = sol.value(U);

end

%% Local functions
function total_cost = local_compute_cost(X, U, params)
% - acceleration cost
cost_hpr_acc = [diff(X(6,:))*diff(X(6,:))'
    diff(X(8,:))*diff(X(8,:))'
    diff(X(10,:))*diff(X(10,:))'];

cost_fcmd     = [U(3,:)*U(3,:)'
    U(4,:)*U(4,:)'
    U(5,:)*U(5,:)'
    U(6,:)*U(6,:)'];

cost_hpr_disp = [X(5,:)*X(5,:)'
    X(7,:)*X(7,:)'
    X(9,:)*X(9,:)'];

cost_hpr_vel  = [X(6,:)*X(6,:)'
    X(8,:)*X(8,:)'
    X(10,:)*X(10,:)'];

cost_lat      =  [X(3,:)*X(3,:)'
    (X(2,:)-params.opti.XY(1))*(X(2,:)-params.opti.XY(1))'
    U(1,:)*U(1,:)'];

cost_long     = [(X(4,:)-params.opti.vref)*(X(4,:)-params.opti.vref)'
    U(2,:)*U(2,:)'];

total_cost    =  params.opti.weights.cost_hpr_acc * cost_hpr_acc ...
    + params.opti.weights.cost_fcmd * cost_fcmd ...
    + params.opti.weights.cost_hpr_disp * cost_hpr_disp ...
    + params.opti.weights.cost_hpr_vel * cost_hpr_vel ...
    + params.opti.weights.cost_lat * cost_lat ...
    + params.opti.weights.cost_long * cost_long;
end

