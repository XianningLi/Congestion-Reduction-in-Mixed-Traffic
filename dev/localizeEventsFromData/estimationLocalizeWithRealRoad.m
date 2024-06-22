clear all
close all
clc

load('Fenn_Lanes_EB_resampled_100x100mm.mat')
Z = synced_data';
% Z = Z(10:35, 1800:2300)-0.01; %510-710
Z = Z(10:35, 29000:30000)-0.01;
gridSize = 0.1;
% [Z]=movAvg2D(Z,2,2);
Nx = size(Z,2)*gridSize;
Ny = size(Z,1)*gridSize;

x = linspace(0, Nx, round(Nx/(gridSize)));
y = linspace(0, Ny, round(Ny/(gridSize)));
[X, Y] = meshgrid(x, y);
mesh(X, Y, Z)
daspect([1 1 1]) % adjust the display ratio of each axis
% surf(X,Y,Z,'FaceColor','interp')
% ix = find(imregionalmax(Z));
% hold on
% plot3(X(ix),Y(ix),Z(ix),'r*','MarkerSize',24)

s.X = X;
s.Y = Y;
s.Z = Z;
s.label = 'realData';
s.initX = [X(1,21) mean(Y(:,11)) 0 3 Z(11,21) 0 0 0 0 0];
s.gridSize = gridSize;
s.Nx = Nx;
s.Ny = Ny;
s.decayRate = [0.5 0.5];
s.space = [5 5];
writeScenarioToJson(s, 'fennLanesEB_X2900m3000m_Y1m3p5m.json')

xb = linspace(1, Nx, round(Nx/(s.space(1)*gridSize)));   % select x positions for basis
yb = linspace(1, Ny, round(Ny/(s.space(2)*gridSize)));   % select y positions for basis

[xxb, yyb] = meshgrid(xb, yb); % get locations of basis functions in a grid
decayRate = s.decayRate;                              % define decay rate of basis (0.5 0.5 for a 20m long road)

%% solve by least squares
params.road.X = X;
params.road.Y = Y;
params.road.Z = Z;
params.road.xxb = xxb;
params.road.yyb = yyb;
params.road.decayRate = decayRate;

[alpha, RBFBasis] = getAlpha(params);  % get the basis weights

Zest = RBFBasis*alpha;  % compute estimated road profile
% for i=1:size(X,1)
%     for j = 1:size(X,2)
%         Zest(i,j) = rbfBasisMat(decayRate, X(i,j), Y(i,j), xp, yp)*alpha;
%     end
% end
Zest = reshape(Zest, round(Ny/gridSize), round(Nx/gridSize));

%% localize potholes

detectmin = 0;
pointCloud = -s.Z;
Zest1 = zeros(size(s.Z));
count = 1;
while ~isempty(detectmin)
    [detectmin, I1] = min(min(pointCloud));
    [~, I2] = min(pointCloud(:,I1));

    if detectmin <= -0.03

        %         D = log(pointCloud(I2-1,I1-1)/(detectmin))/( (X(1,I1-1)-X(1,I1))^2+(Y(I2-1,1)-Y(I2,1))^2 );  % enough for detecting events
        %         fpot = @(X,Y) detectmin*exp(D*(X-X(1,I1)).^2+D*(Y-Y(I2,1)).^2);
        A = [(X(1,I1-1)-X(1,I1))^2 0
            0 (Y(I2,1)-Y(I2+1,1))^2];

        b = [log(pointCloud(I2+1,I1-1)/(detectmin))
            log(pointCloud(I2,I1)/(detectmin))];

        D = A\b;
        fpot = @(X,Y) detectmin*exp(D(1)*(X-X(1,I1)).^2+D(2)*(Y-Y(I2,1)).^2);
        Zest1 = Zest1 + fpot(X, Y);
        pointCloud = Zest1+s.Z;
        minVec(count) = detectmin;
        decayVec(:,count) = D;
        centroids(:, count) = [I1;I2];
        count = count + 1;
        pointCloud = -pointCloud;

    else
        detectmin = [];
    end
end

%% Visualization

figure
mesh(X, Y, Z)
title('actual')
daspect([1 1 1]) % adjust the display ratio of each axis

figure
mesh(X, Y, -Zest1)
title('estimated by localization')
daspect([1 1 1]) % adjust the display ratio of each axis

figure
mesh(X, Y, Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
title('estimated')

figure
mesh(X, Y, Zest-Z )
daspect([1 1 1]) % adjust the display ratio of each axis
title('error')

figure
mesh(X, Y, -Zest1-Z )
daspect([1 1 1]) % adjust the display ratio of each axis
title('localization error')