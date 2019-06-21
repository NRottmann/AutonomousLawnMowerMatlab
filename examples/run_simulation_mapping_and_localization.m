% Example 05:
% Simulate the robot (driving along the boundary lines), afterwards
% generate a map estimate based on the simulated data with parameter
% optimization. We then try to localize the robot in this map
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_05.mat';  
load(map);
pose = [0; 0; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Follow the boundary line
T = 3000;       % Simulation time in seconds
startPose = 0;  % Choose a random start pose
[controlUnit,path,estPath] = controlUnit.wallFollowing(T,startPose);

%% Generate map estimate from odometry data
optimize.loopClosure = true;
optimize.mapping = true;
[controlUnit,mappingResults] = controlUnit.mapping(estPath,optimize);

%% Plot the map
figure;
plot(mappingResults.estMap.x,mappingResults.estMap.y)
xlabel('x in meter')
ylabel('y in meter')
title('Map Estimate')

%% Post Process the map
comparisonMode = 4;
[controlUnit,comparisonResults] = controlUnit.compare(comparisonMode);

%% Use global localization - Careful of the map used here. The directions of the DPs is important (CCW)
maxT = 1000;
N = 1;
localizationError = zeros(N,1);
[controlUnit,localizationResults] = controlUnit.globalLocalization(maxT,1,false);
if localizationResults.foundPosition
    figure;
    plot(controlUnit.EstPolyMap.x,controlUnit.EstPolyMap.y)
    hold on
    plot(controlUnit.PolyMap.x,controlUnit.PolyMap.y)
    plot(localizationResults.pose(1),localizationResults.pose(2),'g*')
    plot(localizationResults.estPose(1),localizationResults.estPose(2),'r*')
    xlabel('x in meter')
    ylabel('y in meter')
    axis('equal')
else
    disp('Global localization failed!')
    figure;
    plot(controlUnit.EstPolyMap.x,controlUnit.EstPolyMap.y)
    hold on
    plot(controlUnit.PolyMap.x,controlUnit.PolyMap.y)
    plot(localizationResults.path(1,:),localizationResults.path(2,:),'g')
    plot(localizationResults.pose(1),localizationResults.pose(2),'g*')
    xlabel('x in meter')
    ylabel('y in meter')
    axis('equal')
end



