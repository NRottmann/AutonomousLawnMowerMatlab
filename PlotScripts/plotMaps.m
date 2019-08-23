%% Script for plotting the maps
clear all
close all
clc

%% Load the maps
map1 = load('map_7.mat');           % symmetric
map2 = load('map_6.mat');           % curved
map3 = load('map_5.mat');           % apartment

%% Plot the maps
width = 8.2;        % centimeter for a half page site
height = 4.0;

h1 = figure(1);
set(h1, 'Units','centimeters','Position', [1 1 0.33*width height])
plot(map1.polyMap.x,map1.polyMap.y)
set(gca ,'FontSize' ,10) ;
% xlabel('K','Interpreter','latex')
% ylabel('$$\mathcal{L}$$','Interpreter','latex')
axis([map1.polyMap.XWorldLimits,map1.polyMap.YWorldLimits])
scalebar
box off
axis off

h1 = figure(2);
set(h1, 'Units','centimeters','Position', [1 1 0.33*width height])
plot(map2.polyMap.x,map2.polyMap.y)
set(gca ,'FontSize' ,10) ;
% xlabel('K','Interpreter','latex')
% ylabel('$$\mathcal{L}$$','Interpreter','latex')
axis([map2.polyMap.XWorldLimits,map2.polyMap.YWorldLimits])
scalebar
box off
axis off

h1 = figure(3);
set(h1, 'Units','centimeters','Position', [1 1 0.33*width height])
plot(map3.polyMap.x,map3.polyMap.y)
set(gca ,'FontSize' ,10) ;
% xlabel('K','Interpreter','latex')
% ylabel('$$\mathcal{L}$$','Interpreter','latex')
axis([map3.polyMap.XWorldLimits,map3.polyMap.YWorldLimits])
scalebar
box off
axis off