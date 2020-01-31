% Example 02:
% Simulate the robot (driving along the boundary lines), afterwards
% generate a map estimate 

clear all
close all
clc

%% Choose the map and starting pose
map = 'map_5.mat';
load(map);

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,[0; 0; 0]);

%% Follow the boundary line
[controlUnit,path,estPath] = controlUnit.wallFollowing(2000,0);

%% Plot the estimated path
figure,
plot(estPath(1,:),estPath(2,:))
title('Estimated Path')
xlabel('meter')
ylabel('meter')

%% Generate map estimate from odometry data
optimize.loopClosure = true;
optimize.mapping = 2;
mode.loopClosure = 1;
mode.mapping = 2;
[controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize,mode);

%% Plot the map estimate
figure,
plot(mappingResults.closedDP(1,:),mappingResults.closedDP(2,:))
title('Map Estimate')
xlabel('meter')
ylabel('meter')

%% Compare estimated map with groundtruth
comparisonResults = controlUnit.compare(6);

%% Plot the map estimate
figure,
plot(comparisonResults.PolyMap.x,comparisonResults.PolyMap.y)
hold on
plot(comparisonResults.EstPolyMap.x,comparisonResults.EstPolyMap.y)
title('Estimated Map')
xlabel('meter')
ylabel('meter')
legend('Groundtruth','Map Estimate')

%% Use second comparison method, first we have to generate the groundtruth DPs
groundTruthDP = path(1:2,mappingResults.DP_indices);
groundTruthDP_cut = groundTruthDP(:,mappingResults.cut_indices);
graphRelationsResults = graphRelations(mappingResults.cutDP,groundTruthDP_cut);

disp(graphRelationsResults.errorTrans)
disp(graphRelationsResults.errorRot)
