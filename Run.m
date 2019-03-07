%% Script for running the simulation environment
clear all
close all
clc

%% Choose the map and starting pose
map_name = 'map_01.mat';  
load(map_name);
pose = [0; 0; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Run the code
% Do some wall following
[controlUnit,path,estPath] = controlUnit.wallFollowing(10000,0);

%% Generate map estimate from odometry data
[controlUnit,mappingResults] = controlUnit.mapping(estPath);

%% Comparison between maps
comparisonResults = controlUnit.compare(3);

%% Use global localization - Careful of the map used here. The directions of the DPs is important (CCW) TODO: Solve this
controlUnit.Pose = generateStartPose(polyMap);      % Choose random starting pose dor demonstration
[controlUnit,localizationResults] = controlUnit.globalLocalization(1000);
e = norm(localizationResults.pose(1:2) - localizationResults.estPose(1:2))

%% Complete Coverage with particle filter localization
[controlUnit,coverageResults] = controlUnit.completeCoverage(1000,2);

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