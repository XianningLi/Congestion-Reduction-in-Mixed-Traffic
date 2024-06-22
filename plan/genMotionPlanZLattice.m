function [x, u, totalCost, searchTimeZ] = genMotionPlanZLattice(params)
% Main motion planning function that creates XYZ motion plans

import casadi.*

N = params.opti.N-1; % number of steps

opti = casadi.Opti(); % creating a casadi optimization object
X = opti.variable(6,N+1); 
U = opti.variable(4,N); 

% constraints
% Initial contidion
initialVals = params.veh.initialVals;
opti.subject_to(X(:,1) == initialVals(5:end))


% dynamic constraints
f = @(x, u, k) [
            x(2); 

            (1/params.veh.m)*(getCornerForce(params).fFL([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            +getCornerForce(params).fFR([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            +getCornerForce(params).fRL([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            +getCornerForce(params).fRR([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u]));

            x(4);

            (1/params.veh.Jb)*(-params.veh.l1*getCornerForce(params).fFL([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            -params.veh.l1*getCornerForce(params).fFR([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            +params.veh.l2*getCornerForce(params).fRL([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            +params.veh.l2*getCornerForce(params).fRR([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u]));

            x(6);

            (1/params.veh.Jg)*(params.veh.w*getCornerForce(params).fFL([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            -params.veh.w*getCornerForce(params).fFR([params.xyPlan.x(:,k);x],  [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            +params.veh.w*getCornerForce(params).fRL([params.xyPlan.x(:,k);x], [params.xyPlan.u(k); params.veh.accel(k) ;u])...
            -params.veh.w*getCornerForce(params).fRR([params.xyPlan.x(:,k);x],  [params.xyPlan.u(k); params.veh.accel(k) ;u]));]; %x(1) = x, x(2) = y, x(3) = theta, x(4) = V, x(5) = z1, x(6) = z2, u(1) = k, u(2) = acc., u(3) = fa


dt = params.opti.dt;

% objective 
total_cost = local_compute_cost(X, U, params);
opti.minimize(total_cost);
for k = 1:N
    x_next = X(:,k) + dt(k)*f(X(:,k), U(:,k), k);
    opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% control limit
opti.subject_to(params.ctrlLims.uLim(3:end,1)<=U<=params.ctrlLims.uLim(3:end,2)) 

% Avoid twist force
opti.subject_to([1 -1 -1 1]*U==0)

% state limit

% set limit on z displacement for four corners
opti.subject_to( params.veh.susTravLim(1,1) <= getCornerCoords(params).XYZ.zFL([params.xyPlan.x;X]) - getCornerCoords(params).road.rFL([params.xyPlan.x]) <= params.veh.susTravLim(1,2))
opti.subject_to( params.veh.susTravLim(2,1) <= getCornerCoords(params).XYZ.zFR([params.xyPlan.x;X]) - getCornerCoords(params).road.rFR([params.xyPlan.x]) <= params.veh.susTravLim(2,2))
opti.subject_to( params.veh.susTravLim(3,1) <= getCornerCoords(params).XYZ.zRL([params.xyPlan.x;X]) - getCornerCoords(params).road.rRL([params.xyPlan.x]) <= params.veh.susTravLim(3,2))
opti.subject_to( params.veh.susTravLim(4,1) <= getCornerCoords(params).XYZ.zRR([params.xyPlan.x;X]) - getCornerCoords(params).road.rRR([params.xyPlan.x]) <= params.veh.susTravLim(4,2))

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend

tic
sol = opti.solve();   % actual solve
searchTimeZ = toc;


x = sol.value(X);
u = sol.value(U);
totalCost = local_compute_cost(x, u, params);

end

%% Local functions
function total_cost = local_compute_cost(X, U, params)
% - acceleration cost
cost_hpr_acc = [diff(X(2,:))*diff(X(2,:))'
    diff(X(4,:))*diff(X(4,:))'
    diff(X(6,:))*diff(X(6,:))'];

cost_fcmd     = [U(1,:)*U(1,:)'
    U(2,:)*U(2,:)'
    U(3,:)*U(3,:)'
    U(4,:)*U(4,:)'];

cost_hpr_disp = [X(1,:)*X(1,:)'
    X(3,:)*X(3,:)'
    X(5,:)*X(5,:)'];

cost_hpr_vel  = [X(2,:)*X(2,:)'
    X(4,:)*X(4,:)'
    X(6,:)*X(6,:)'];

cost_long     =  ( params.xyPlan.x(4,:)-params.opti.vref )*(  params.xyPlan.x(4,:)-params.opti.vref )';
                   

total_cost    =  params.opti.weights.cost_hpr_acc * cost_hpr_acc ...
    + params.opti.weights.cost_fcmd * cost_fcmd ...
    + params.opti.weights.cost_hpr_disp * cost_hpr_disp ...
    + params.opti.weights.cost_hpr_vel * cost_hpr_vel; 
    + params.opti.weights.cost_long(1)*cost_long;
    
end

