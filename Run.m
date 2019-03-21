%% Script for running the simulation environment
clear all
close all
clc

%% Choose the map and starting pose
map_name = 'map_garden03.mat';  
load(map_name);
pose = [0; 0; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Load data
load('Garden03.mat')
estPath = pose;

% %% Run the code
% % Do some wall following
% [controlUnit,path,estPath] = controlUnit.wallFollowing(2000,0);

%% Use Bayesian Opitmization
optimization = Optimization(controlUnit);
param = optimization.BayesianOptimization(estPath);

% %% Generate map estimate from odometry data
% [controlUnit,mappingResults] = controlUnit.mapping(estPath);
% 
% %% Comparison between maps
% comparisonResults = controlUnit.compare(4);

% %% Use global localization - Careful of the map used here. The directions of the DPs is important (CCW) TODO: Solve this
% controlUnit.Pose = generateStartPose(polyMap);      % Choose random starting pose dor demonstration
% [controlUnit,localizationResults] = controlUnit.globalLocalization(1000);
% e = norm(localizationResults.pose(1:2) - localizationResults.estPose(1:2))
% 
% %% Complete Coverage with particle filter localization
% [controlUnit,coverageResults] = controlUnit.completeCoverage(1000,2);
% 
% %% Plot some results
% figure(2)
% subplot(1,2,1)
% plot(polyMap.x,polyMap.y)
% hold on
% plot(coverageResults.path(1,:),coverageResults.path(2,:))
% subplot(1,2,2)
% plot(polyMap.x,polyMap.y)
% hold on
% plot(coverageResults.estPath(1,:),coverageResults.estPath(2,:))



%% PLot 3D data
x = param.X(1,:);
y = param.X(2,:);
z = param.f;

xlin = linspace(min(x),max(x),100);
ylin = linspace(min(y),max(y),100);
[X,Y] = meshgrid(xlin,ylin);

Z = griddata(x,y,z,X,Y,'cubic');

mesh(X,Y,Z)
axis tight; hold on
plot3(x,y,z,'.','MarkerSize',15)