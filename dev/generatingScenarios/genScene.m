function s = genScene(paramsGenScene)

Nx = paramsGenScene.Nx;                   % Nx = length of road in x direction (m)
Ny = paramsGenScene.Ny;                   % Ny = length of road in y direction (m)
numPot = paramsGenScene.numPot;           % number of potholes
numBump = paramsGenScene.numBump;         % number of road bumps
noiseLevel = paramsGenScene.noiseLevel;
% numRectDip = paramsGenScene.numRectDip;   % number of rect dips
gridSize = 0.1; % 10 cm                   % size of each grid

x = linspace(0, Nx, round(Nx/(gridSize)));   % select x positions for basis
y = linspace(0, Ny, round(Ny/(gridSize)));   % select y positions for basis

[X, Y] = meshgrid(x, y);

if numPot == 0 && numBump == 0
    error('Error: numPot > 0 or numBump >0')
elseif numPot == 0
    [Z, dimLoc] = bumpSurface(X, Y, Nx, Ny, gridSize, numBump, noiseLevel);
    label = 'road bump';
elseif numBump == 0
    [Z, dimLoc] = potholeSurface(X, Y, Nx, Ny, gridSize, numPot, noiseLevel);
    label = 'pothole';
else
    [ZBump, bumpDimLoc] = bumpSurface(X, Y, Nx, Ny, gridSize, numBump, noiseLevel);
    [ZPot, potDimLoc] = potholeSurface(X, Y, Nx, Ny, gridSize, numPot, noiseLevel);
    dimLoc.pot = potDimLoc;
    dimLoc.bump = bumpDimLoc;
    Z = ZBump + ZPot;
    label = 'pothole+bump';
end

s.X = X;
s.Y = Y;
s.Z = Z;
s.label = label;
s.gridSize = gridSize;
s.Nx = Nx;
s.Ny = Ny;
s.dimLoc = dimLoc;

end

%% helper function for Pothole
function [Z, potDimLoc] = potholeSurface(X, Y, Nx, Ny, gridSize, numPot, noiseLevel)

Depth = (0.0762 - 0.04)*rand(numPot,1)+0.04; % 0.0762 (3 inch) is the max pothole depth, 0.0254 (1 inch) is the min pothole depth
Width = (30 - 20)*rand(numPot,1)+20; % decay rate
indx = randi([200 700], 1, numPot); % the index gives the area on the grid to place a pothole
indy = randi([10 25], 1, numPot); % the index gives the area on the grid to place a pothole
xy = [X(1,indx);
    Y(indy, 1)'];
Z = zeros(size(X));
for i = 1:numPot
    Z = Z-Depth(i)*exp(-Width(i)*(X-xy(1,i)).^2-Width(i)*(Y-xy(2,i)).^2);
end
Z = Z + noiseLevel*rand(round(Ny/gridSize), round(Nx/gridSize));
potDimLoc.depth = Depth;
potDimLoc.width = Width;
potDimLoc.indx = indx;
potDimLoc.indy = indy;
potDimLoc.centriods = xy;
end

%% helper function for Road bump
function [Z, bumpDimLoc] = bumpSurface(X, Y, Nx, Ny, gridSize, numBump, noiseLevel)

Depth = (0.127-0.0762)*rand(numBump,1)+0.0762; % 0.0762 (3 inch) is the min bump height, 0.127 (5 inch) is the max bump height
Width = (30 - 20)*rand(numBump,1)+20; % decay rate
indx = randi([200 700], 1, numBump); % the index gives the area on the grid to place a pothole
indy = randi([10 25], 1, numBump); % the index gives the area on the grid to place a pothole
xy = [X(1,indx);
    Y(indy, 1)'];
Z = 0;
for i = 1:numBump
    Z = Z+Depth(i)*exp(-Width(i)*(X-xy(1,i)).^2-0*(Y-xy(2,i)).^2);
end
Z = Z + noiseLevel*rand(round(Ny/gridSize), round(Nx/gridSize));
bumpDimLoc.depth = Depth;
bumpDimLoc.width = Width;
bumpDimLoc.indx = indx;
bumpDimLoc.indy = indy;
bumpDimLoc.centriods = xy;
end