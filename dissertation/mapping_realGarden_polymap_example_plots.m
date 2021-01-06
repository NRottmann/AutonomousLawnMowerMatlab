% Mapping Scheme for real environments
% Generate plots

% Params: out.l_min = 0.1;
%         out.e_max = 0.001;
%         out.l_nh = 15;
%         out.c_max = 0.06;
%         out.gamma1 = 1;
%         out.gamma2 = 0.001;

clear all
close all
clc

%% Load the data
data = 'garden01.mat';
load(data)                  % Load the real data
load(['map_' data]);        % Load groundtruth map

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,zeros(3,1));

%% Generate map estimate from odometry data
optimize.loopClosure.flag = 0;
optimize.loopClosure.plotting.flag = 0;
optimize.loopClosure.plotting.width = 16.2/3;    % centimeter for a page site
optimize.loopClosure.plotting.height = 5.0;
optimize.mapping = 0;

mode.loopClosure = 1;       % 1: Scan alignment (ECMR Paper), 2: ICP
mode.mapping = 2;           % 1: ECMR Paper, 2: ICP as proposed in IROS Workshop Paper

plotting.flag = true;
plotting.width = 16.2/3;    % centimeter for a page site
plotting.height = 5.0;

[controlUnit,mappingResults] = controlUnit.mapping(pose,optimize,mode,plotting);

%% Compare estimated map with groundtruth
[controlUnit,comparisonResults] = controlUnit.compare(6);

%% Plot the data

% Cutted graph
h = figure;
set(h, 'Units','centimeters','Position', [1 1 plotting.width plotting.height])
plot(comparisonResults.cutDP(1,:),comparisonResults.cutDP(2,:),'LineWidth',1.5)
hold on
plot(comparisonResults.cutDP(1,:),comparisonResults.cutDP(2,:),'r.','MarkerSize',5)
set(gca ,'FontSize' ,10)
scalebar;
box off
axis off
legend('Graph','DPs')
legend boxoff

% Closed map estimate
h = figure;
set(h, 'Units','centimeters','Position', [1 1 plotting.width plotting.height])
plot(comparisonResults.closedDP(1,:),comparisonResults.closedDP(2,:),'LineWidth',1.5)
hold on
plot(comparisonResults.closedDP(1,:),comparisonResults.closedDP(2,:),'r.','MarkerSize',5)
set(gca ,'FontSize' ,10)
scalebar;
box off
axis off

% Comparison
poly1 = polyshape(polyMap.x,polyMap.y,'Simplify',false);
poly2 = polyshape(comparisonResults.turnedEstPolyMap.x,comparisonResults.turnedEstPolyMap.y,'Simplify',false);
polyout1 = subtract(poly1,poly2);
polyout2 = subtract(poly2,poly1);

h = figure;
set(h, 'Units','centimeters','Position', [1 1 plotting.width plotting.height])
plot(polyMap.x,polyMap.y,'Color',[0.8500 0.3250 0.0980],'LineWidth',1.5)
hold on
plot(comparisonResults.turnedEstPolyMap.x,comparisonResults.turnedEstPolyMap.y,'Color',[0 0.4470 0.7410],'LineWidth',1.5)
hold on
plot(polyout1,'LineStyle','none','FaceColor',[0.6350, 0.0780, 0.1840])
hold on
plot(polyout2,'LineStyle','none','FaceColor',[0.6350, 0.0780, 0.1840])
set(gca ,'FontSize' ,10)
scalebar;
box off
axis off
legend('Groundtruth','Estimate','Deviation')
legend boxoff
