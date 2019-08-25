% Example 01:
% Generate a map of the environment based on real data, improve mapping
% parameters and evaluate map results
clear all
close all
clc

%% Choose the data set and load the data, also the groundtruth map, if it exists
data = 'garden04.mat';
load(data)                  % Load real data
% load(['map_' data]);        % Load groundtruth map
load('map_garden03.mat')

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,zeros(3,1));

%% Generate map estimate from odometry data
optimize.loopClosure = true;
optimize.mapping = false;
[controlUnit,mappingResults] = controlUnit.mapping(pose,optimize);

%% Compare estimated map with groundtruth
% comparisonResults = controlUnit.compare(4);

%% Plot
figure;
plot(mappingResults.cutDP(1,:),mappingResults.cutDP(2,:))

figure;
plot(mappingResults.estMap.x,mappingResults.estMap.y)
