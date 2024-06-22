% function coords = cornerCoords(x, params)
function coords = getCornerCoords(params)

% compute x,y,z coordinates for four corners
coords.XYZ.xFL = @(x) x(1,:)+params.veh.l1*cos(x(3,:))-params.veh.w*sin(x(3,:));
coords.XYZ.yFL = @(x) x(2,:)+params.veh.w*cos(x(3,:))+params.veh.l1*sin(x(3,:));
coords.XYZ.zFL = @(x) x(5,:)-params.veh.l1*x(7,:)+params.veh.w*x(9,:);

coords.XYZ.xFR = @(x) x(1,:)+params.veh.l1*cos(x(3,:))+params.veh.w*sin(x(3,:));
coords.XYZ.yFR = @(x) x(2,:)-params.veh.w*cos(x(3,:))+params.veh.l1*sin(x(3,:));
coords.XYZ.zFR = @(x) x(5,:)-params.veh.l1*x(7,:)-params.veh.w*x(9,:);

coords.XYZ.xRL = @(x) x(1,:)-params.veh.l2*cos(x(3,:))-params.veh.w*sin(x(3,:));
coords.XYZ.yRL = @(x) x(2,:)+params.veh.w*cos(x(3,:))-params.veh.l2*sin(x(3,:));
coords.XYZ.zRL = @(x) x(5,:)+params.veh.l2*x(7,:)+params.veh.w*x(9,:);

coords.XYZ.xRR = @(x) x(1,:)-params.veh.l2*cos(x(3,:))+params.veh.w*sin(x(3,:));
coords.XYZ.yRR = @(x) x(2,:)-params.veh.w*cos(x(3,:))-params.veh.l2*sin(x(3,:));
coords.XYZ.zRR = @(x) x(5,:)+params.veh.l2*x(7,:)-params.veh.w*x(9,:);

% compute the road profiles for each side
coords.road.rFL = @(x) params.road.RoadData.fun1([coords.XYZ.xFL(x); coords.XYZ.yFL(x)]);
coords.road.rRL = @(x) params.road.RoadData.fun1([coords.XYZ.xRL(x); coords.XYZ.yRL(x)]);
coords.road.rRR = @(x) params.road.RoadData.fun1([coords.XYZ.xRR(x); coords.XYZ.yRR(x)]);
coords.road.rFR = @(x) params.road.RoadData.fun1([coords.XYZ.xFR(x); coords.XYZ.yFR(x)]);

% compute the derivative of road profiles for each side
coords.road.drFL = @(x) params.road.RoadData.dr([coords.XYZ.xFL(x); coords.XYZ.yFL(x); x(3,:); x(4,:)]);
coords.road.drRL = @(x) params.road.RoadData.dr([coords.XYZ.xRL(x); coords.XYZ.yRL(x); x(3,:); x(4,:)]);
coords.road.drRR = @(x) params.road.RoadData.dr([coords.XYZ.xRR(x); coords.XYZ.yRR(x); x(3,:); x(4,:)]);
coords.road.drFR = @(x) params.road.RoadData.dr([coords.XYZ.xFR(x); coords.XYZ.yFR(x); x(3,:); x(4,:)]);

end