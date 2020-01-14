% Example 02:
% Simulate the robot (driving along the boundary lines), afterwards
% generate a map estimate based on the simulated data without parameter
% optimization and show the map estimate.
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_5.mat';  
load(map);
pose = [0; 0; 0];

N = 20;
E = zeros(N,1);

E_trans = zeros(N,1);
E_rot = zeros(N,1);

Param = cell(N,1);

for i=1:1:N
    
%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Follow the boundary line
T = 2000;       % Simulation time in seconds
startPose = 0;  % Choose a random start pose
[controlUnit,path,estPath] = controlUnit.wallFollowing(T,startPose);

%% Generate map estimate from odometry data
optimize.loopClosure = false;
optimize.mapping = 1;
[controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize);

%% Post Process the map
[controlUnit,comparisonResults] = controlUnit.compare(6);

E(i) = comparisonResults.error;

%% Use second error metric
groundTruthDP = path(1:2,mappingResults.DP_indices);
groundTruthDP_cut = groundTruthDP(:,mappingResults.cut_indices);
graphRelationsResults = graphRelations(mappingResults.cutDP,groundTruthDP_cut);

E_trans(i) = graphRelationsResults.errorTrans;
E_rot(i) = graphRelationsResults.errorRot;

Param{i} = mappingResults.param;

end