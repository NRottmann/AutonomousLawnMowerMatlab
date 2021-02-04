% test something
close all
clear all
clc

%% Load
random = load('results_map_10_mode1.mat');
optimal = load('results_map_10_mode2.mat');
nnccpp = load('results_map_10_mode3.mat');

%% Plot
figure
plot(random.results.travelledDist,random.results.trueCoverage)
hold on
plot(optimal.results.travelledDist,optimal.results.trueCoverage)
plot(nnccpp.results.travelledDist,nnccpp.results.trueCoverage)
legend('random','optimal','nnccpp')
