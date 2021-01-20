% Example 04:
% Simulate the robot mowing the lawn. Thereby we use a partcile filter for
% pose estimation.
clear all
close all
clc

%% Choose the map and starting pose
map = 'map_1.mat'; % the map to mow
load(map);
pose = [0.05; 0.05; 0];

%% Initialize the control unit
% controlUnit = ControlUnit(polyMap,pose);
controlUnit = ControlUnit(gridMap,pose);

%% Complete Coverage with particle filter localization
mode = 2;       % Random Walk(1), NNCCPP(2), coverage with random(3), coverage with nnccpp(4), wallfollower(5)
particleMap = false;
tic
% first parameter: if mode with coverage, then the coverage to reach, else the time in seconds
% second is the mode
% third is if the NNCCPP should run with the particlemap or not
[controlUnit,coverageResults] = controlUnit.completeCoverage(500, mode, particleMap); 
t = toc
coverageResults.time = t;
coverageResults.polyMap = polyMap;
% save('NilsOmaPlanned99.mat', 'coverageResults')
% end

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
if mode == 2 || mode == 4
    figure()
    surf(coverageResults.neuralActivity');
    title('Neural Activity');
    figure()
    surf(coverageResults.externalInput');
    title('External Input');
end
figure()
surf(coverageResults.coverageMap')
grid on
xlabel('x in Zellen')
ylabel('y in Zellen')
title('Abdeckungskarte')
colorbar
pbaspect([1 1 1])
figure()
surf(coverageResults.particleCoverageMap')
grid on
xlabel('x in Zellen')
ylabel('y in Zellen')
title('Abdeckungskarte')
colorbar
pbaspect([1 1 1])
