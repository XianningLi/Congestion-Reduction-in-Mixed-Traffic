clear all
close all
clc

%% define parameters
filename = 'bumpTestCase5.json';
data = readScenarioFromJson(filename);

xb = linspace(1, data.Nx, round(data.Nx/(data.space(1)*data.gridSize)));   % select x positions for basis
yb = linspace(1, data.Ny, round(data.Ny/(data.space(2)*data.gridSize)));   % select y positions for basis

[xxb, yyb] = meshgrid(xb, yb);
decayRate = data.decayRate;        % define decay rate of basis

%% solve by least squares
params.road.X = data.X;
params.road.Y = data.Y;
params.road.Z = data.Z;
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
Zest = reshape(Zest, round(data.Ny/data.gridSize), round(data.Nx/data.gridSize));
%% localize Roadbumps

detectmax = 0;
pointCloud = data.Z;
Zest1 = zeros(size(data.Z));
count = 1;
a = 1;
while ~isempty(detectmax)
[detectmax, I1] = max(max(pointCloud));

if detectmax >= 0.07
    D = log(pointCloud(1,I1+1)/detectmax)/( (data.X(1,I1+1)-data.X(1,I1))^2 );
    if abs(D) ~= inf
        fpot = @(X) detectmax*exp(D*(X-data.X(1,I1)).^2);
        Zest1 = Zest1 + fpot(data.X);
        pointCloud = -Zest1 + data.Z; % make erroras the new point cloud
        minVec(count) = detectmax;
        decayVec(count) = D;
        centroids(1, count) = I1;
        count = count + 1;
    else
        warning("Decay rate is -inf")
    end
else
    detectmax = [];
end

end

%% Visualization

figure
mesh(params.road.X, params.road.Y, params.road.Z)
title('actual')
daspect([1 1 1]) % adjust the display ratio of each axis

figure
mesh( params.road.X, params.road.Y, Zest)
daspect([1 1 1]) % adjust the display ratio of each axis
title('estimated')

figure
mesh( params.road.X, params.road.Y, Zest1)
daspect([1 1 1]) % adjust the display ratio of each axis
title('by estimating the decay rate')

figure
mesh( params.road.X, params.road.Y, Zest1-params.road.Z )
daspect([1 1 1]) % adjust the display ratio of each axis
title('error')



