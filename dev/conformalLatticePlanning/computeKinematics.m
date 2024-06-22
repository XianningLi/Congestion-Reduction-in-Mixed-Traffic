function bicycleStates = computeKinematics(p, s, bestPrev, params)

% get the polyomial parameters [a b c d] from the parameterization
% parameters pi's
polyParams = getPolyParams(p, params);

% get curvature
kappa = polyParams(1,:) + polyParams(2,:).*s + polyParams(3,:).*s.^2 + polyParams(4,:).*s.^3;

% get heading
theta =  getTheta(polyParams, s, bestPrev);

% get x,y which are Fresnel integrals. Compute x,y using Simpson's method
x = bestPrev(1) + (s/24).*( cos(getTheta(polyParams,0, bestPrev)) + 4*cos(getTheta(polyParams,s/8, bestPrev))...
    + 2*cos(getTheta(polyParams,2*s/8, bestPrev)) + 4*cos(getTheta(polyParams,3*s/8, bestPrev))...
    + 2*cos(getTheta(polyParams,4*s/8, bestPrev)) + 4*cos(getTheta(polyParams,5*s/8, bestPrev))...
    + 2*cos(getTheta(polyParams,6*s/8, bestPrev)) + 4*cos(getTheta(polyParams,7*s/8, bestPrev))...
    + cos(getTheta(polyParams,s, bestPrev)) );

y = bestPrev(2) + (s/24).*( sin(getTheta(polyParams,0, bestPrev)) + 4*sin(getTheta(polyParams,s/8, bestPrev))...
    + 2*sin(getTheta(polyParams,2*s/8, bestPrev)) + 4*sin(getTheta(polyParams,3*s/8, bestPrev))...
    + 2*sin(getTheta(polyParams,4*s/8, bestPrev)) + 4*sin(getTheta(polyParams,5*s/8, bestPrev))...
    + 2*sin(getTheta(polyParams,6*s/8, bestPrev)) + 4*sin(getTheta(polyParams,7*s/8, bestPrev))...
    + sin(getTheta(polyParams,s, bestPrev)) );

% return the computed states
bicycleStates = [x; y; theta; kappa];

end

function theta = getTheta(polyParams, s, bestPrev)

theta = bestPrev(3) + polyParams(1,:).*s + polyParams(2,:).*(s.^2)/2 + polyParams(3,:).*(s.^3)/3 + polyParams(4,:).*(s.^4)/4;

end
