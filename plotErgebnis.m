close all, clear all, clc;

load('planning10noError.mat');

figure;
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.path(1,:),coverageResults.path(2,:))
pbaspect([1 1 1])
set(gca,'visible','off')

time = length(coverageResults.coverages)/20;
time = time/60

figure;
surf(coverageResults.neuralActivity')