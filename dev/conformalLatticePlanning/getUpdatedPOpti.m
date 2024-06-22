function [solTime, updatedP] = getUpdatedPOpti(p0, params, targets, bestPrev)

import casadi.*

opti = casadi.Opti(); % creating a casadi optimization object
[m,n] = size(p0);
p = opti.variable(m,n);

% objective
total_cost = getTotalCost(p, params, targets, bestPrev);
opti.minimize(total_cost);

% constraints on curvature
if params.flag == "XY"
    opti.subject_to(-0.19 <= getKappa(p, p(end,:)/3, params) <= 0.19)
    opti.subject_to(-0.19 <= getKappa(p, p(end,:)*(2/3), params) <= 0.19)
    opti.subject_to(getKappa(p, p(end,:), params) == targets(end, :) )
else
%     opti.subject_to(-0.19 <= getDist(p, p(end,:)/4, flag) <= 0.19)
%     opti.subject_to(-0.19 <= getDist(p, p(end,:)*(1/2), flag) <= 0.19)
%     opti.subject_to(-0.19 <= getDist(p, p(end,:)*(3/4), flag) <= 0.19)
end

opti.set_initial(p, p0); % initialize the solver

% set solver options to supress printing
solver_options= struct;
solver_options.ipopt.print_level = 0;
solver_options.print_time =0;
solver_options.verbose = 0;
solver_options.ipopt.sb ='yes';

opti.solver('ipopt', solver_options); % set numerical backend

tic
sol = opti.solve();   % actual solve
solTime = toc; % get the solver time

updatedP = sol.value(p);

end


%% Local functions
function totalcost = getTotalCost(p, params, targets, bestPrev)

if params.flag == "XY"
% get the bicycle states
bicycleStates = computeKinematics(p, p(end,:), bestPrev, params);

% get the total cost for deviation from the targets
totalcost =  params.xyPlan.weights_spiral*[(bicycleStates(1,:)-targets(1,:))*(bicycleStates(1,:)-targets(1,:))' 
                                (bicycleStates(2,:)-targets(2,:))*(bicycleStates(2,:)-targets(2,:))'
                                (bicycleStates(3,:)-targets(3,:))*(bicycleStates(3,:)-targets(3,:))'];
else



end


end

function bendingEnergy = getBendingEnergy(p)

polyParams = getPolyParams(p); % polyParams = [a b c d]

n = 20;
delta = p(end,:)/n;
intervals = 1:n;
intervals = repmat(intervals',1,numel(delta));
xi = intervals'.*delta';

bendingEnergy = sum(sum( (( polyParams(1,:)'+polyParams(2,:)'.*xi+polyParams(3,:)'.*xi.^2+polyParams(4,:)'.*xi.^3 ).^2).*delta' , 2 ));

end

function kappa = getKappa(p, s, params)

    polyParams = getPolyParams(p, params);
    kappa = polyParams(1,:) + polyParams(2,:).*s + polyParams(3,:).*s.^2 + polyParams(4,:).*s.^3;

end

