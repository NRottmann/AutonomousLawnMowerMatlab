% Example 02:
% Simulate the robot (driving along the boundary lines), afterwards
% generate a map estimate based on the simulated data without parameter
% optimization and show the map estimate.
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_05.mat';  
load(map);
pose = [0; 0; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Follow the boundary line
T = 3000;       % Simulation time in seconds
startPose = 0;  % Choose a random start pose
[controlUnit,path,estPath] = controlUnit.wallFollowing(T,startPose);

%% Generate map estimate from odometry data
optimize.loopClosure = false;
optimize.mapping = false;
[controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize);

%% Plot the map estimate
figure(1)
plot(mappingResults.estMap.x,mappingResults.estMap.y)