% Example 04:
% Simulate the robot mowing the lawn. Thereby we use a partcile filter for
% pose estimation.
for x = 2:1:10
clear all\x
close all
clc

%% Choose the map and starting pose
map = 'map_01.mat';  
load(map);
pose = [0.1; 0.1; 0];

%% Initialize the control unit
controlUnit = ControlUnit(polyMap,pose);

%% Complete Coverage with particle filter localization
mode = 3;       % Random Walk(1), NNCCPP(2), coverage with random(3), coverage with nnccpp(4), wallfollower(5)
particleMap = true;
    tic
    [controlUnit,coverageResults] = controlUnit.completeCoverage(0.9, mode, particleMap);
    t = toc
    coverageResults.time = t;
    coverageResults.polyMap = polyMap;
    name = strcat('random_', num2str(x))
    save(name, 'coverageResults')
end
% save('random25.mat', 'coverageResults')

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
