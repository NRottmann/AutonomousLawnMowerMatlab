% Mapping Scheme for real environments
% Evaluate for all real data

% Map 03: c_max=0.3, l_nh=30, gamma1=1, gamma2=0.001
% Map 04: c_max=0.21, l_nh=50, gamma1=0.001, gamma2=100

clear all
close all
clc

%% Load the data
data = 'garden04.mat';
load(data)                      % Load the real data

compData = false;
if isfile(['map_' data])
    load(['map_' data]);        % Load groundtruth map
    compData = true;
end
    

%% Initialize the control unit
if compData
    controlUnit = ControlUnit(polyMap,zeros(3,1));
else
    controlUnit = ControlUnit([],zeros(3,1));
end

%% Generate map estimate from odometry data
optimize.loopClosure.flag = 0;
optimize.loopClosure.plotting.flag = 0;
optimize.loopClosure.plotting.width = 16.2/3;    % centimeter for a page site
optimize.loopClosure.plotting.height = 5.0;
optimize.mapping = 0;

mode.loopClosure = 1;       % 1: Scan alignment (ECMR Paper), 2: ICP
mode.mapping = 2;           % 1: ECMR Paper, 2: ICP as proposed in IROS Workshop Paper

plotting.flag = false;
plotting.width = 16.2/3;    % centimeter for a page site
plotting.height = 5.0;

[controlUnit,mappingResults] = controlUnit.mapping(pose,optimize,mode,plotting);

%% Compare estimated map with groundtruth
if compData
    [controlUnit,comparisonResults] = controlUnit.compare(6);
end

%% Plot the data

% Orginal Data
h = figure;
set(h, 'Units','centimeters','Position', [1 1 plotting.width plotting.height])
plot(pose(1,:),pose(2,:),'LineWidth',1.5)
set(gca ,'FontSize' ,10)
scalebar;
box off
axis off

% Comparison
if compData
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
else
    h = figure;
    set(h, 'Units','centimeters','Position', [1 1 plotting.width plotting.height])
    plot(mappingResults.cutDP(1,:),mappingResults.cutDP(2,:),'Color',[0 0.4470 0.7410],'LineWidth',1.5)
    set(gca ,'FontSize' ,10)
    scalebar;
    box off
    axis off
end
