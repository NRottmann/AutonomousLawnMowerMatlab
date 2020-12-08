% Script for simulating the mapping procedure of the autonmous lawn mower
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_1.mat';
load(map);

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,[0; 0; 0]);

%% Follow the boundary line
[controlUnit,path,estPath] = controlUnit.wallFollowing(1500,0);

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
