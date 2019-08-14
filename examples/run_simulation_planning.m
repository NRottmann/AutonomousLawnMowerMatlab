% Example 04:
% Simulate the robot mowing the lawn. Thereby we use a partcile filter for
% pose estimation.
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_01.mat';  
load(map);
pose = [0.1; 0.1; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Complete Coverage with particle filter localization
mode = 2;       % Random Walk(1), NNCCPP(2), coverage with random(3), coverage with nnccpp(4)
[controlUnit,coverageResults] = controlUnit.completeCoverage(0.9, 4);
coverageResults.polyMap = polyMap;

% save('random7200.mat', 'coverageResults')

%% Plot some results
figure()
subplot(1,2,1)
plot(polyMap.x,polyMap.y)
hold on
plot(coverageResults.path(1,:),coverageResults.path(2,:))
subplot(1,2,2)
plot(polyMap.x,polyMap.y)
hold on
plot(coverageResults.estPath(1,:),coverageResults.estPath(2,:))
hold off
figure()
surf(coverageResults.groundTruth');
title('groundTruth');
figure()
surf(coverageResults.particleCoverageMap');
title('ParticleCoverageMap');
figure()
surf(coverageResults.coverageMap');
title('CoverageMap');
figure()
particle = coverageResults.particleCoverageMap;
particle(particle < controlUnit.Threshhold) = 0;
particle(particle >= controlUnit.Threshhold) = 1;
surf(particle');
title('ParticleCoverageMapThreshhold');
figure()
coverage = coverageResults.coverageMap;
coverage(coverage < controlUnit.Threshhold) = 0;
coverage(coverage >= controlUnit.Threshhold) = 1;
surf(coverage');
title('CoverageMapThreshhold');
if mode == 2
    figure()
    surf(coverageResults.neuralActivity');
    title('Neural Activity');
    figure()
    surf(coverageResults.externalInput');
    title('External Input');
end
