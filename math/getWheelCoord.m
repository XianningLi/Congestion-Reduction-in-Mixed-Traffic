function [x,y,z] = getWheelCoord(x0, y0, z0, params)

r = params.viz.wheelRadius;  % radius of tire
w = params.viz.wheelWidth;   % width of tire

numPoints = 21; 
theta = linspace(0,2*pi, numPoints);

z = z0 + [r*sin(theta); r*sin(theta)] + r;
x = x0 + [r*cos(theta); r*cos(theta)];
y = y0 + [-ones(1, numPoints)*w/2
           ones(1, numPoints)*w/2];
end