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

N = 20;     % Number if iterations
M = 4;      % Number of methods

% Define settings
settings = cell(M,1);
settings{1}.optimize.loopClosure = 0;
settings{1}.optimize.mapping = 0;
settings{1}.mode.loopClosure = 1;
settings{1}.mode.mapping = 1;
settings{2}.optimize.loopClosure = 0;
settings{2}.optimize.mapping = 0;
settings{2}.mode.loopClosure = 1;
settings{2}.mode.mapping = 2;
settings{3}.optimize.loopClosure = 1;
settings{3}.optimize.mapping = 0;
settings{3}.mode.loopClosure = 1;
settings{3}.mode.mapping = 2;
settings{4}.optimize.loopClosure = 1;
settings{4}.optimize.mapping = 1;
settings{4}.mode.loopClosure = 1;
settings{4}.mode.mapping = 2;
% settings{5}.optimize.loopClosure = 1;
% settings{5}.optimize.mapping = 0;
% settings{5}.mode.loopClosure = 1;
% settings{5}.mode.mapping = 1;
% settings{6}.optimize.loopClosure = 1;
% settings{6}.optimize.mapping = 0;
% settings{6}.mode.loopClosure = 1;
% settings{6}.mode.mapping = 2;

E = zeros(N,M);
E_trans = zeros(N,M);
E_rot = zeros(N,M);
Param = cell(N,M);

for i=1:1:N
    
%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Follow the boundary line
T = 2000;       % Simulation time in seconds
startPose = 0;  % Choose a random start pose
[controlUnit,path,estPath] = controlUnit.wallFollowing(T,startPose);

%% Copy control units
controlUnitCell = cell(M,1);
for j=1:1:M
    controlUnitCell{j} = controlUnit;
end

%% Generate map estimate from odometry data
for j=1:1:M
    [controlUnitCell{j},mappingResults] = controlUnitCell{j}.mapping(estPath,settings{j}.optimize,settings{j}.mode);
    
    % Second error metric
    groundTruthDP = path(1:2,mappingResults.DP_indices);
    groundTruthDP_cut = groundTruthDP(:,mappingResults.cut_indices);
    graphRelationsResults = graphRelations(mappingResults.cutDP,groundTruthDP_cut);
    E_trans(i,j) = graphRelationsResults.errorTrans;
    E_rot(i,j) = graphRelationsResults.errorRot;
    Param{i,j} = mappingResults.param;
end

%% Post Process the map
for j=1:1:M
    if isempty(controlUnitCell{j}.EstPolyMap)
        E(i,j) = inf;
    else
        [controlUnitCell{j},comparisonResults] = controlUnitCell{j}.compare(6);
        E(i,j) = comparisonResults.error;
    end
end

disp(i)

end

%% Save the data
% Get Parameters
out = get_config('mapping');
mappingParam = out;
out = get_config('odometryModelNoise');
odomParam = out;
save('data2.mat','E','E_rot','E_trans','mappingParam','odomParam','Param','settings')