function plotTraj(X, U, params, createGif, filename, fig)
if nargin < 6
    fig = figure(7);
end
this_axes = axes('Parent', fig);
steadyStateHeight = params.viz.steadyStateHeight;

coords = getCornerCoords(params);

% calculate the coords for each waypoint along the trajectory
x = [coords.XYZ.xFL(X); coords.XYZ.xRL(X); coords.XYZ.xRR(X); coords.XYZ.xFR(X)];
y = [coords.XYZ.yFL(X); coords.XYZ.yRL(X); coords.XYZ.yRR(X); coords.XYZ.yFR(X)];
z = [coords.XYZ.zFL(X); coords.XYZ.zRL(X); coords.XYZ.zRR(X); coords.XYZ.zFR(X)] + steadyStateHeight;

% Road height at each waypoint
zr = [coords.road.rFL(X);
    coords.road.rRL(X);
    coords.road.rRR(X);
    coords.road.rFR(X)];

%% Start plotting road and car
mesh(params.road.X,params.road.Y,params.road.Zest)
hold on

% Wheel path

for ct = 1:4
    plot3(x(ct, :), y(ct, :), zr(ct, :), 'k-');
    hold on
end

% initialize animation
p = fill3(x(:,1), y(:,1), z(:,1), 'g');

p11 = line([x(1,1) x(1,1)], ...
    [y(1,1) y(1,1)], ...
    [z(1,1) zr(1,1)+params.viz.wheelRadius], 'Color','red', 'Linewidth', 2);
[xw,yw,zw] = getWheelCoord(x(1,1), y(1,1), zr(1, 1), params);
p21 = surface(xw,yw,zw);
p12 = line([x(2,1) x(2,1)], ...
    [y(2,1) y(2,1)], ...
    [z(2,1) zr(2,1)+params.viz.wheelRadius], 'Color','red', 'Linewidth', 2);
[xw,yw,zw] = getWheelCoord(x(2,1), y(2,1), zr(2, 1), params);
p22 = surface(xw,yw,zw);

p13 = line([x(3,1) x(3,1)], ...
    [y(3,1) y(3,1)], ...
    [z(3,1) zr(3,1)+params.viz.wheelRadius], 'Color','red', 'Linewidth', 2);
[xw,yw,zw] = getWheelCoord(x(3,1), y(3,1), zr(3, 1), params);
p23 = surface(xw,yw,zw);

p14 = line([x(4,1) x(4,1)], ...
    [y(4,1) y(4,1)], ...
    [z(4,1) zr(4,1)+params.viz.wheelRadius], 'Color','red', 'Linewidth', 2);
[xw,yw,zw] = getWheelCoord(x(4,1), y(4,1), zr(4, 1), params);
p24 = surface(xw,yw,zw);
xlim([min(params.road.X(:)), max(params.road.X(:))]);
ylim([min(params.road.Y(:)), max(params.road.Y(:))]);
zlim([min(params.road.Zest(:)), max(params.road.Zest(:))+steadyStateHeight])
daspect([1 1 1])

% do animation
% Way points (downsampled)
for i = 2:size(x,2)

    % plot body
    p.XData = x(:,i);
    p.YData = y(:,i);
    p.ZData = z(:,i);

    % Plot each wheels
    p11.XData = [x(1,i) x(1,i)];
    p11.YData = [y(1,i) y(1,i)];
    p11.ZData = [z(1,i) zr(1,i)+params.viz.wheelRadius];

    p12.XData = [x(2,i) x(2,i)];
    p12.YData = [y(2,i) y(2,i)];
    p12.ZData = [z(2,i) zr(2,i)+params.viz.wheelRadius];

    p13.XData = [x(3,i) x(3,i)];
    p13.YData = [y(3,i) y(3,i)];
    p13.ZData = [z(3,i) zr(3,i)+params.viz.wheelRadius];

    p14.XData = [x(4,i) x(4,i)];
    p14.YData = [y(4,i) y(4,i)];
    p14.ZData = [z(4,i) zr(4,i)+params.viz.wheelRadius];

    [xw,yw,zw] = getWheelCoord(x(1,i), y(1,i), zr(1,i), params);
    p21.XData = xw;
    p21.YData = yw;
    p21.ZData = zw;

    [xw,yw,zw] = getWheelCoord(x(2,i), y(2,i), zr(2,i), params);
    p22.XData = xw;
    p22.YData = yw;
    p22.ZData = zw;

    [xw,yw,zw] = getWheelCoord(x(3,i), y(3,i), zr(3,i), params);
    p23.XData = xw;
    p23.YData = yw;
    p23.ZData = zw;

    [xw,yw,zw] = getWheelCoord(x(4,i), y(4,i), zr(4,i), params);
    p24.XData = xw;
    p24.YData = yw;
    p24.ZData = zw;

    if createGif == 1
        exportgraphics(this_axes,strcat(filename,'.gif'),'Append',true);
    end
    xlim([min(params.road.X(:)), max(params.road.X(:))]);
    ylim([min(params.road.Y(:)), max(params.road.Y(:))]);
    zlim([min(params.road.Zest(:)), max(params.road.Zest(:))+steadyStateHeight])
    drawnow
    pause(0.07)
end

% Annotations
hold off
xlim([min(params.road.X(:)), max(params.road.X(:))]);
ylim([min(params.road.Y(:)), max(params.road.Y(:))]);
zlim([min(params.road.Zest(:)), max(params.road.Zest(:))+steadyStateHeight]);
daspect([1 1 1])

end
