%% Script for plotting the maps
clear all
close all
clc

%% Load the maps
data = load('plotSet.mat');

%% Plot the maps
width = 8.2;        % centimeter for a half page site
height = 4.0;

h1 = figure(1);
set(h1, 'Units','centimeters','Position', [1 1 0.5*width height])
plot(data.model_points(1,data.idx_vertices),data.model_points(2,data.idx_vertices))
hold on
plot(data.model_points(1,data.idx_vertices),data.model_points(2,data.idx_vertices),'r.')
set(gca ,'FontSize' ,10) ;
xlabel('K','Interpreter','latex')
ylabel('$$\mathcal{L}$$','Interpreter','latex')
% axis([map1.polyMap.XWorldLimits,map1.polyMap.YWorldLimits])
box off
axis off
scalebar

h2 = figure(2);
set(h2, 'Units','centimeters','Position', [1 1 0.25*width height])
plot(data.modelSet(1,:),data.modelSet(2,:),'b.')
hold on
plot(data.testSet(1,:),data.testSet(2,:),'r.')
plot(data.modelSet(1,data.N_NH+1),data.modelSet(2,data.N_NH+1),'k*')
set(gca ,'FontSize' ,10) ;
xlabel('K','Interpreter','latex')
ylabel('$$\mathcal{L}$$','Interpreter','latex')
% axis([map1.polyMap.XWorldLimits,map1.polyMap.YWorldLimits])
box off
axis off
scalebar

h3 = figure(3);
set(h3, 'Units','centimeters','Position', [1 1 0.25*width height])
plot(data.modelSet(1,:),data.modelSet(2,:),'b.')
hold on
plot(data.testSetNew(1,:),data.testSetNew(2,:),'r.')
plot(data.modelSet(1,data.N_NH+1),data.modelSet(2,data.N_NH+1),'k*')
set(gca ,'FontSize' ,10) ;
xlabel('K','Interpreter','latex')
ylabel('$$\mathcal{L}$$','Interpreter','latex')
% axis([map1.polyMap.XWorldLimits,map1.polyMap.YWorldLimits])
box off
axis off
scalebar
