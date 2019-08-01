% Example 02:
% Simulate the robot (driving along the boundary lines), afterwards
% generate a map estimate based on the simulated data without parameter
% optimization and show the map estimate.
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_19.mat';  
load(map);
pose = [0; 0; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Follow the boundary line
T = 12000;       % Simulation time in seconds
startPose = 0;  % Choose a random start pose
[controlUnit,path,estPath] = controlUnit.wallFollowing(T,startPose);

%% Generate map estimate from odometry data
optimize.loopClosure = true;
optimize.mapping = true;
[controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize);

%% Compare estimated map with groundtruth
comparisonResults = controlUnit.compare(4);

%% Plots
figure,
plot(estPath(1,:),estPath(2,:))
title('Estimated Path')

figure;
plot(mappingResults.estMap.x,mappingResults.estMap.y)
title('Map Estimate')
