% Script for simulating the mapping procedure of the autonmous lawn mower
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_1.mat';
load(map);

%% Parameters
optimize.loopClosure.flag = 1;
optimize.loopClosure.plotting.flag = 0;
optimize.loopClosure.plotting.width = 16.2/3;    % centimeter for a page site
optimize.loopClosure.plotting.height = 5.0;
optimize.mapping = 1;

mode.loopClosure = 1;       % 1: Scan alignment (ECMR Paper), 2: ICP
mode.mapping = 2;           % 1: ECMR Paper, 2: ICP as proposed in IROS Workshop Paper

plotting.flag = false;
plotting.width = 16.2/3;    % centimeter for a page site
plotting.height = 5.0;

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,[0; 0; 0]);

%% Follow the boundary line
[controlUnit,path,estPath] = controlUnit.wallFollowing(1000,0);

%% Plot the estimated path
figure
plot(path(1,:),path(2,:))
title('Path')
xlabel('meter')
ylabel('meter')

figure
plot(estPath(1,:),estPath(2,:))
title('Estimated Path')
xlabel('meter')
ylabel('meter')

%% Generate map estimate from odometry data
[controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize,mode,plotting);

%% Compare estimated map with groundtruth
[controlUnit,comparisonResults] = controlUnit.compare(6);

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

%% Use comparison method, first we have to generate the groundtruth DPs
groundTruthDP = path(1:2,mappingResults.DP_indices);
groundTruthDP_cut = groundTruthDP(:,mappingResults.cut_indices);

unoptimizedDP = estPath(1:2,mappingResults.DP_indices);
unoptimizedDP_cut = unoptimizedDP(:,mappingResults.cut_indices);

% Graph relations without optimzation
graphRelationsResultsUnoptimized = graphRelations(unoptimizedDP_cut,groundTruthDP_cut);

% Graph relations with optimization
graphRelationsResultsOptimized = graphRelations(mappingResults.cutDP,groundTruthDP_cut);

errorTransProgress = (graphRelationsResultsUnoptimized.errorTrans -  ...
                        graphRelationsResultsOptimized.errorTrans) / graphRelationsResultsUnoptimized.errorTrans;
errorRotProgress = (graphRelationsResultsUnoptimized.errorRot -  ...
                        graphRelationsResultsOptimized.errorRot) / graphRelationsResultsUnoptimized.errorRot;
                    
errorProgress = (graphRelationsResultsUnoptimized.error -  ...
                        graphRelationsResultsOptimized.error) / graphRelationsResultsUnoptimized.error;
                    
disp(errorTransProgress)
disp(errorRotProgress)
disp(errorProgress)
