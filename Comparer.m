clear all, close all, clc;

results = 'random20.mat';

load(results);
mapAbs = mapAbsolute(coverageResults.polyMap, 10);

p = immse(coverageResults.groundTruth, coverageResults.particleCoverageMap)
c = immse(coverageResults.groundTruth, coverageResults.coverageMap)
tmp = coverageResults.groundTruth;
tmp(mapAbs==0) = 0;
% coverage = sum(tmp==1)/sum(mapAbs==1)

figure()
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.path(1,:),coverageResults.path(2,:))
xlim([-1 , 11])
ylim([-1 , 11])
grid on
xlabel('x in Meter')
ylabel('y in Meter')
title('Karte')
pbaspect([1 1 1])

figure()
surf(coverageResults.groundTruth')
grid on
xlabel('x in Zellen')
ylabel('y in Zellen')
title('Abdeckungsmatrix')
colorbar
pbaspect([1 1 1])