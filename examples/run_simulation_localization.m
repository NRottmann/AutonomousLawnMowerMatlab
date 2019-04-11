% Example 03:
% Simulate the robot by letting it find its position using the global
% localization algorithm and print out the error after global localization
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_05.mat';  
load(map);
pose = [0; 0; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Use global localization - Careful of the map used here. The directions of the DPs is important (CCW)
maxT = 1000;
controlUnit.Pose = generateStartPose(polyMap);      % Choose random starting pose
[controlUnit,localizationResults] = controlUnit.globalLocalization(maxT);
if localizationResults.foundPosition
    e = norm(localizationResults.pose(1:2) - localizationResults.estPose(1:2))
else
    disp('Global localization failed!')
end

