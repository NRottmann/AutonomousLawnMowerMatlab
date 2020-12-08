% Script for simulating the mapping procedure of the autonmous lawn mower

clear all
close all
clc

%% Choose the map and starting pose
map = 'map_1.mat';
load(map);

%% Initialize the control unit
controlUnit = ControlUnit(gridMap,[0; 0; 0]);

%% Follow the boundary line
[controlUnit,path,estPath] = controlUnit.wallFollowing(1500,0);

%% Plot the estimated path
figure,
plot(estPath(1,:),estPath(2,:))
title('Estimated Path')
xlabel('meter')
ylabel('meter')

%% Generate map estimate from odometry data
optimize.loopClosure = 0;
optimize.mapping = 0;
mode.loopClosure = 1;       % 1: Scan alignment (ECMR Paper), 2: ICP
mode.mapping = 1;           % 1: ECMR Paper, 2: ICP as proposed in IROS Workshop Paper
[controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize,mode);

%% Plot the map estimate
figure
plot(mappingResults.cutDP(1,:),mappingResults.cutDP(2,:))
title('Cut DP')
xlabel('meter')
ylabel('meter')

figure
plot(mappingResults.closedDP(1,:),mappingResults.closedDP(2,:))
title('Closed DP')
xlabel('meter')
ylabel('meter')

% %% Compare estimated map with groundtruth
% comparisonResults = controlUnit.compare(6);
% 
% %% Plot the map estimate
% figure,
% plot(comparisonResults.PolyMap.x,comparisonResults.PolyMap.y)
% hold on
% plot(comparisonResults.EstPolyMap.x,comparisonResults.EstPolyMap.y)
% title('Estimated Map')
% xlabel('meter')
% ylabel('meter')
% legend('Groundtruth','Map Estimate')

% %% Use second comparison method, first we have to generate the groundtruth DPs
% groundTruthDP = path(1:2,mappingResults.DP_indices);
% groundTruthDP_cut = groundTruthDP(:,mappingResults.cut_indices);
% graphRelationsResults = graphRelations(mappingResults.cutDP,groundTruthDP_cut);
% 
% disp(graphRelationsResults.errorTrans)
% disp(graphRelationsResults.errorRot)
