%% Clear everything
close all;
clear all;
clc;

%% Choose the data set
load('Garden01.mat')

%% Do the simulation
[results] = realData(pose,1);

%% Plot
figure(1)
subplot(2,3,1)
plot(results.odometryPose(1,:),results.odometryPose(2,:))
title('Estimated Pose')
subplot(2,3,2)
plot(results.tutorialGraph(1,:),results.tutorialGraph(2,:));
title('Tutorial')
subplot(2,3,3)
plot(results.adjustedTutorialGraph(1,:),results.adjustedTutorialGraph(2,:));
title('adjusted Tutorial')
subplot(2,3,4)
plot(results.lagoGraph(1,:),results.lagoGraph(2,:));
title('Lago')
subplot(2,3,5)
show(results.matlabGraph)
title('Matlab')
