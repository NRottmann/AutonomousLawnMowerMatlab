% Example 04:
% Simulate the robot mowing the lawn. Thereby we use a partcile filter for
% pose estimation.
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_05.mat';  
load(map);
pose = [0; 0; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Complete Coverage with particle filter localization
mode = 1;       % Random Walk
[controlUnit,coverageResults] = controlUnit.completeCoverage(100,mode);

%% Plot some results
figure(2)
subplot(1,2,1)
plot(polyMap.x,polyMap.y)
hold on
plot(coverageResults.path(1,:),coverageResults.path(2,:))
subplot(1,2,2)
plot(polyMap.x,polyMap.y)
hold on
plot(coverageResults.estPath(1,:),coverageResults.estPath(2,:))

