
function generateGifNew(createGif, filename, x, params, fig)
coords = cornerCoords(params);
if nargin < 5
    fig = figure();
end
this_axes = axes('Parent', fig);
mesh(params.road.X, params.road.Y, params.road.Z)
daspect([1 1 1]) 
hold on
% plot3(x(1,:), x(2,:), x(5,:), 'o-', 'linewidth', 0.5)
x1 = [coords.XYZ.xFL(x(:,1)) coords.XYZ.xRL(x(:,1)) coords.XYZ.xRR(x(:,1)) coords.XYZ.xFR(x(:,1))];
y1 = [coords.XYZ.yFL(x(:,1)) coords.XYZ.yRL(x(:,1)) coords.XYZ.yRR(x(:,1)) coords.XYZ.yFR(x(:,1))];
z1 = [coords.XYZ.zFL(x(:,1)) coords.XYZ.zRL(x(:,1)) coords.XYZ.zRR(x(:,1)) coords.XYZ.zFR(x(:,1))];
p = fill3(x1, y1, z1, 1);
for i = 1:100
    x1 = [coords.XYZ.xFL(x(:,i)) coords.XYZ.xRL(x(:,i)) coords.XYZ.xRR(x(:,i)) coords.XYZ.xFR(x(:,i))];
    y1 = [coords.XYZ.yFL(x(:,i)) coords.XYZ.yRL(x(:,i)) coords.XYZ.yRR(x(:,i)) coords.XYZ.yFR(x(:,i))];
    z1 = [coords.XYZ.zFL(x(:,i)) coords.XYZ.zRL(x(:,i)) coords.XYZ.zRR(x(:,i)) coords.XYZ.zFR(x(:,i))];
    p.XData = x1;
    p.YData = y1;
    p.ZData = z1;
    %     daspect([1 1 1])
    drawnow
    if createGif == 1
        exportgraphics(this_axes,strcat(filename,'.gif'),'Append',true);
    end
    pause(.05)

    leftLen = sqrt( (coords.XYZ.xRL(x(:,i))-coords.XYZ.xFL(x(:,i)))^2+(coords.XYZ.yRL(x(:,i))-coords.XYZ.yFL(x(:,i)))^2 );
    rightLen = sqrt( (coords.XYZ.xRR(x(:,i))-coords.XYZ.xFR(x(:,i)))^2+(coords.XYZ.yRR(x(:,i))-coords.XYZ.yFR(x(:,i)))^2 );
    leftWidth = sqrt( (coords.XYZ.xRL(x(:,i))-coords.XYZ.xRR(x(:,i)))^2+(coords.XYZ.yRL(x(:,i))-coords.XYZ.yRR(x(:,i)))^2 );
    rightWidth = sqrt( (coords.XYZ.xFL(x(:,i))-coords.XYZ.xFR(x(:,i)))^2+(coords.XYZ.yFL(x(:,i))-coords.XYZ.yFR(x(:,i)))^2 );
    lengthWidth = [leftLen leftWidth rightLen rightWidth]
end