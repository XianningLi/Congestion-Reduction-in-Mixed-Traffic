clear all
clc
close all

paramsGenScene.Nx = 100;                                    % Nx = length of road in x direction (m)
paramsGenScene.Ny = 3.7;                                   % Ny = length of road in y direction (m)
numPot = [1 2 3];
numBump = [1 2];
vel = [0.1 0.2 0.3];

%% generate vanilla test case
x = linspace(0, 100, round(100/(0.1)));
y = linspace(0, 4, round(2.5/(0.1)));
[X, Y] = meshgrid(x, y);

s.X = X;
s.Y = Y;
s.Z = zeros(size(X));
s.label = 'vanilla'; % label of the data
s.initX = [X(1,21) Y(11,1) 0 25 s.Z(11,21) 0 0 0 0 0]; % initial value
s.gridSize = 0.1; % gridsize
s.Nx = 100;  % length of x
s.Ny = 2.5; % length of y
s.decayRate = [0.5 0.5]; % decay rate of basis functions
s.space = [5 5]; % spacing between basis function locations
writeScenarioToJson(s, 'testcase0.json')

%% pothole and small bumps
count = 1;
for i = 1:length(numPot)
        for k = 1:length(vel)
            paramsGenScene.numPot = numPot(i);                                 % number of potholes
            paramsGenScene.numBump = 0;                                % number of road bumps
            paramsGenScene.noiseLevel = 0.01;                              % set zero if you want the road surface to be smooth 
            filename = strcat('potTestCase', num2str(count), '.json');
            s = genScene(paramsGenScene);
            s.initX = [s.X(1,41) s.Y(11,1) 0 vel(k) s.Z(11,41) 0 0 0 0 0];
            s.decayRate = [0.5 0.5];
            s.space = [5 5];
            writeScenarioToJson(s, filename)
            s.Z = -s.Z; % small bumps
            filename = strcat('smallBumpTestCase', num2str(count), '.json');
            writeScenarioToJson(s, filename)
            count = count + 1;
        end
end


%% road bump and road dips
count = 1;
for j = 1:length(numBump)
    for k = 1:length(vel)
        paramsGenScene.numPot = 0;                                 % number of potholes
        paramsGenScene.numBump = numBump(j);                                % number of road bumps
        paramsGenScene.noiseLevel = 0.01;                              % set zero if you want the road surface to be smooth 
        filename = strcat('bumpTestCase', num2str(count), '.json');
        s = genScene(paramsGenScene);
        s.initX = [s.X(1,41) s.Y(11,1) 0 vel(k) s.Z(11,41) 0 0 0 0 0];
        s.decayRate = [0.5 0.5];
        s.space = [5 5];
        writeScenarioToJson(s, filename)
        s.Z = -s.Z;
        filename = strcat('roaddipsTestCase', num2str(count), '.json');
        writeScenarioToJson(s, filename)
        count = count + 1;
    end
end

%% road pothole and bump
count = 1;
for i = 1:length(numPot)
    for j = 1:length(numBump)
        for k = 1:length(vel)
            paramsGenScene.numPot = numPot(i);                                 % number of potholes
            paramsGenScene.numBump = numBump(j);                                % number of road bumps
            paramsGenScene.noiseLevel = 0;                              % set zero if you want the road surface to be smooth 
            filename = strcat('potBumpNoiseZeroTestCase', num2str(count), '.json');
            s = genScene(paramsGenScene);
            s.initX = [s.X(1,41) s.Y(11,1) 0 vel(k) s.Z(11,41) 0 0 0 0 0];
            s.decayRate = [0.5 0.5];
            s.space = [5 5];
            writeScenarioToJson(s, filename)
            count = count + 1;
        end
    end
end
%% Visualization
% plotScenario('testcase32.json')
