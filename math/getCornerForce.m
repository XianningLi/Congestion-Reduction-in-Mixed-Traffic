function forces = getCornerForce(params)


coords = getCornerCoords(params);

% compute the total force in four sides
forces.fFL = @(x,u) params.veh.k(1)*(coords.road.rFL(x)-x(5,:)+params.veh.l1*x(7,:)-params.veh.w*x(9,:))+params.veh.c(1)*(coords.road.drFL(x)-x(6,:)+params.veh.l1*x(8,:)-params.veh.w*x(10,:))+u(3,:);
forces.fFR = @(x,u) params.veh.k(2)*(coords.road.rFR(x)-x(5,:)+params.veh.l1*x(7,:)+params.veh.w*x(9,:))+params.veh.c(2)*(coords.road.drFR(x)-x(6,:)+params.veh.l1*x(8,:)+params.veh.w*x(10,:))+u(4,:);
forces.fRL = @(x,u) params.veh.k(3)*(coords.road.rRL(x)-x(5,:)-params.veh.l2*x(7,:)-params.veh.w*x(9,:))+params.veh.c(3)*(coords.road.drRL(x)-x(6,:)-params.veh.l2*x(8,:)-params.veh.w*x(10,:))+u(5,:);
forces.fRR = @(x,u) params.veh.k(4)*(coords.road.rRR(x)-x(5,:)-params.veh.l2*x(7,:)+params.veh.w*x(9,:))+params.veh.c(4)*(coords.road.drRR(x)-x(6,:)-params.veh.l2*x(8,:)+params.veh.w*x(10,:))+u(6,:);



end