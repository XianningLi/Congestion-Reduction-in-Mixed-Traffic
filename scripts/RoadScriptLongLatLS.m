clear all
close all
clc

%% define parameters
filename = 'potBumpNoiseZeroTestCase13.json';
data = readScenarioFromJson(filename);

xb = linspace(1, data.Nx, round(data.Nx/(data.space(1)*data.gridSize)));   % select x positions for basis
yb = linspace(1, data.Ny, round(data.Ny/(data.space(2)*data.gridSize)));   % select y positions for basis

[xxb, yyb] = meshgrid(xb, yb);

%% solve by least squares
params.road.X = data.X;
params.road.Y = data.Y;
params.road.Z = data.Z;
% params.road.Z = movmean(data.Z, size(params.road.X));
params.road.xxb = xxb;
params.road.yyb = yyb;
params.road.decayRate = data.decayRate;

[alpha, RBFBasis] = getAlpha(params);  % get the basis weights

Zest = RBFBasis*alpha;  % compute estimated road profile
% for i=1:size(X,1)
%     for j = 1:size(X,2)
%         Zest(i,j) = rbfBasisMat(decayRate, X(i,j), Y(i,j), xp, yp)*alpha;
%     end
% end
Zest = reshape(Zest, round(data.Ny/data.gridSize), round(data.Nx/data.gridSize));

%% Visualization

figure
mesh(data.X, data.Y, data.Z)
title('actual')
daspect([1 1 1]) % adjust the display ratio of each axis

figure
mesh( params.road.X, params.road.Y, Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
title('estimated')

figure
mesh( params.road.X, params.road.Y, Zest-params.road.Z )
daspect([1 1 1]) % adjust the display ratio of each axis
title('error')



