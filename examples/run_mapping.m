% Example 01:
% Generate a map of the environment based on real data, improve mapping
% parameters and evaluate map results
clear all
close all
clc

%% Choose the data set and load the data, also the groundtruth map, if it exists
data = 'garden01.mat';
load(data)                  % Load real data
% load(['map_' data]);        % Load groundtruth map
load('map_garden01.mat')

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,zeros(3,1));

%% Generate map estimate from odometry data
optimize.loopClosure = true;
optimize.mapping = 1;
[controlUnit,mappingResults] = controlUnit.mapping(pose,optimize);

%% Compare estimated map with groundtruth
% [controlUnit,comparisonResults] = controlUnit.compare(6);

%% Close the map
J = length(mappingResults.cutDP(1,:));
j = 1;
Storage = zeros(J,2);
while ~exist('X_closed') && j <= (J-3)
    X_tmp = mappingResults.cutDP(1:2,j:end) - mappingResults.cutDP(1:2,j);
    S_tmp = zeros(length(X_tmp(1,:)),1);
    for i=1:1:length(X_tmp(1,:))
        S_tmp(i) = norm(X_tmp(:,i));
    end
    [pks, locs] = findpeaks(-S_tmp);
    [~,idx] = max(pks);
    Storage(j,1) = j;
    Storage(j,2) = locs(idx) - j;
    j = j + 1;
end
%% Close
aa = 66;%j
X_closed = [mappingResults.cutDP(:,Storage(aa,1):Storage(aa,1)+Storage(aa,2)+Storage(aa,1)-2),mappingResults.cutDP(:,Storage(aa,1))];
plot(X_closed(1,:),X_closed(2,:))
%% Plot
width = 16.4;        % centimeter for a half page site
height = 0.75 * 0.333 * width;

h1 = figure(1);
set(h1, 'Units','centimeters','Position', [1 1 0.333*width height])
plot(pose(1,:),pose(2,:))
set(gca ,'FontSize' ,10) ;
% xlabel('K','Interpreter','latex')
% ylabel('$$\mathcal{L}$$','Interpreter','latex')
scalebar

box off
axis off 

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

h2 = figure(2);
set(h2, 'Units','centimeters','Position', [1 1 0.333*width height])
plot(X_closed(1,:),X_closed(2,:))
set(gca ,'FontSize' ,10) ;
% xlabel('K','Interpreter','latex')
% ylabel('$$\mathcal{L}$$','Interpreter','latex')
scalebar
box off
axis off  