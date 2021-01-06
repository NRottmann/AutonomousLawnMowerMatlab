% Example 01:
% Generate a map of the environment based on real data, improve mapping
% parameters and evaluate map results
clear all
close all
clc

%% Choose the data set and load the data, also the groundtruth map, if it exists
data = 'garden03.mat';
load(data)                      % Load real data
% load(['map_' data]);          % Load groundtruth map
load('map_garden03.mat')

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,zeros(3,1));

%% Generate map estimate from odometry data
optimize.loopClosure = false;
optimize.mapping = 0;
mode.loopClosure = 1;
[controlUnit,mappingResults] = controlUnit.mapping(pose,optimize,mode);

%% Compare estimated map with groundtruth
[controlUnit,comparisonResults] = controlUnit.compare(6);

%% Plot
% width = 16.4;        % centimeter for a half page site
% height = 0.75 * 0.333 * width;
% 
% h1 = figure(1);
% set(h1, 'Units','centimeters','Position', [1 1 0.333*width height])
% plot(pose(1,:),pose(2,:))
% set(gca ,'FontSize' ,10) ;
% % xlabel('K','Interpreter','latex')
% % ylabel('$$\mathcal{L}$$','Interpreter','latex')
% scalebar
% 
% box off
% axis off 

% h2 = figure(2);
% set(h2, 'Units','centimeters','Position', [1 1 0.333*width height])
% plot(polyMap.x,polyMap.y)
% hold on
% plot(comparisonResults.turnedEstPolyMap.x,comparisonResults.turnedEstPolyMap.y)
% set(gca ,'FontSize' ,10) ;
% % xlabel('K','Interpreter','latex')
% % ylabel('$$\mathcal{L}$$','Interpreter','latex')
% scalebar
% box off
% axis off  
