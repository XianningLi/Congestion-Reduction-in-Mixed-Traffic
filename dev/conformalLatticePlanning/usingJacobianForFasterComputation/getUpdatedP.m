function updatedP = getUpdatedP(p0, target, bestPrev)

maxIter = 200;
tol = 1e-3;
error = 10;
iter = 0;
while (iter <= maxIter)
    bicycleStates = computeKinematics(p0, p0(5), bestPrev)
    error = target - bicycleStates;
    Jacobian = getJacobian(p0, bestPrev);
    stepP = Jacobian\error;
    updatedP = p0 + stepP;
%     p0 = [p0(1);updatedP];
    p0 = updatedP;
    iter = iter + 1
    if (norm(error) < tol)
        norm(error)
        break
    end
end

end