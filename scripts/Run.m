%% Clear everything
close all;
clear all;
clc;

%% Choose the map
map_name = 'map_02.mat';  
load(map_name);

%% Do the simulation
T = 1800;
[results] = simulation(polyMap,T,1);