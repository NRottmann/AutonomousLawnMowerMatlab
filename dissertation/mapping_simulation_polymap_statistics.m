% Script for simulating the mapping procedure of the autonmous lawn mower
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_6.mat';
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

%% Go over n iteration
n = 20;

results = cell(n,1);
params = cell(n,1);

parfor i=1:1:n

    %% Initialize the control unit
    controlUnit = ControlUnit(polyMap,[0; 0; 0]);

    %% Follow the boundary line
    [controlUnit,path,estPath] = controlUnit.wallFollowing(1000,0);

    %% Generate map estimate from odometry data
    [controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize,mode,plotting);
    param{i} = mappingResults.param;

    %% Compare estimated map with groundtruth
    [controlUnit,comparisonResults] = controlUnit.compare(6);
    results{i}.mapComparisonResults = comparisonResults;

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
                        
    results{i}.errorTransProgress = errorTransProgress;
    results{i}.errorRotProgress = errorRotProgress;
    results{i}.errorProgress = errorProgress;
                        
end
                    
%% Store the data
save(['results_',map],'results','param')

mapComp = zeros(n,1);
for i=1:1:n
    mapComp(i) = results{i}.mapComparisonResults.error;
end

disp(mean(mapComp))
disp(std(mapComp))
