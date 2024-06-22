function Jacobian = getJacobian(p, bestPrev)

h = 0.000001;

bicycleStates = computeKinematics(p, p(5), bestPrev);

dBicycleStatesP11 = computeKinematics([p(1)+h p(2) p(3) p(4) p(5)], p(5), bestPrev);
dBicycleStatesP12 = computeKinematics([p(1)-h p(2) p(3) p(4) p(5)], p(5), bestPrev);

dBicycleStatesP21 = computeKinematics([p(1) p(2)+h p(3) p(4) p(5)], p(5), bestPrev);
dBicycleStatesP22 = computeKinematics([p(1) p(2)-h p(3) p(4) p(5)], p(5), bestPrev);

dBicycleStatesP31 = computeKinematics([p(1) p(2) p(3)+h p(4) p(5)], p(5), bestPrev);
dBicycleStatesP32 = computeKinematics([p(1) p(2) p(3)-h p(4) p(5)], p(5), bestPrev);

dBicycleStatesP41 = computeKinematics([p(1) p(2) p(3) p(4)+h p(5)], p(5), bestPrev);
dBicycleStatesP42 = computeKinematics([p(1) p(2) p(3) p(4)-h p(5)], p(5), bestPrev);

dBicycleStatesP51 = computeKinematics([p(1) p(2) p(3) p(4) p(5)+h], p(5)+h, bestPrev);
dBicycleStatesP52 = computeKinematics([p(1) p(2) p(3) p(4) p(5)-h], p(5)-h, bestPrev);

Jacobian = (1/(2*h))*[dBicycleStatesP11-dBicycleStatesP12 dBicycleStatesP21-dBicycleStatesP22 dBicycleStatesP31-dBicycleStatesP32 dBicycleStatesP41-dBicycleStatesP42 dBicycleStatesP51-dBicycleStatesP52];

end