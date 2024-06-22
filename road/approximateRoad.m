function  RoadData = approximateRoad(params)

roadHeight = @(x) params.road.alpha'*RBFBasisLongLat(params.road.decayRate, x(1,:)', x(2,:)', params.road.xxb(:)', params.road.yyb(:)')'; 

drx = @(x) -2*params.road.decayRate(1)*(RBFBasisLongLatDerivX(params.road.decayRate, x(1,:)', x(2,:)', params.road.xxb(:)', params.road.yyb(:)')*params.road.alpha)';
dry = @(x) -2*params.road.decayRate(2)*(RBFBasisLongLatDerivY(params.road.decayRate, x(1,:)', x(2,:)', params.road.xxb(:)', params.road.yyb(:)')*params.road.alpha)';
dr =  @(x) drx(x).*x(4,:).*cos(x(3,:))+dry(x).*x(4,:).*sin(x(3,:)); 

RoadData.fun1 = roadHeight;
RoadData.drx  = drx; % x derivative of pothole 1;
RoadData.dry  = dry; % y derivative of pothole 1;
RoadData.dr   = dr; % time derivative of pothole 1

end