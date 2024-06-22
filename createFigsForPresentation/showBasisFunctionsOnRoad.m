clear all
close all
clc

%% define parameters
Nx = 10;                   % Nx = length of road in x direction (m)
Ny = 3;                   % Ny = length of road in y direction (m)
gridSize = 0.1; % 10 cm                   % size of each grid

x = linspace(1, Nx, round(Nx/(gridSize)));   % select x positions for basis
y = linspace(1, Ny, round(Ny/(gridSize)));   % select y positions for basis

[X, Y] = meshgrid(x, y);

% xb = linspace(1, Nx, round(Nx/(10*gridSize)));   % select x positions for basis
% yb = linspace(1, Ny, round(Ny/(10*gridSize)));   % select y positions for basis

xb = x(1:10:end);   % select x positions for basis
yb = y(1:7:end);   % select y positions for basis

[xxb, yyb] = meshgrid(xb, yb);
decayRate = [50 50];                              % define decay rate of basis

Z = zeros(size(X));
Z(1:7:end, 1:10:end) = 1*ones(length(yb), length(xb));

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

%% Visualization



figure
mesh( X, Y, Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
title('basis functions placed at different coordinates')



