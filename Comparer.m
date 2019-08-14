clear all, close all, clc;

results = 'test.mat';

load(results);
mapAbs = mapAbsolute(coverageResults.polyMap, 5);

figure()
subplot(1,2,1)
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.path(1,:),coverageResults.path(2,:))
subplot(1,2,2)
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.estPath(1,:),coverageResults.estPath(2,:))
hold off

p = immse(coverageResults.groundTruth, coverageResults.particleCoverageMap)
c = immse(coverageResults.groundTruth, coverageResults.coverageMap)
tmp = coverageResults.groundTruth;
tmp(mapAbs==0) = 0;
coverage = sum(tmp==1)/sum(mapAbs==1)

figure()
subplot(1,2,1)
contourf(coverageResults.particleCoverageMap')
colorbar
hold on
plot(coverageResults.polyMap.x.*5+5,coverageResults.polyMap.y.*5+5, 'k', 'LineWidth', 3)
plot(coverageResults.path(1,:).*5+5,coverageResults.path(2,:).*5+5, 'y')
title('ParticleCoverageMap')
hold off
subplot(1,2,2)
contourf(coverageResults.coverageMap')
colorbar
hold on
plot(coverageResults.polyMap.x.*5+5,coverageResults.polyMap.y.*5+5, 'k', 'LineWidth', 3)
plot(coverageResults.path(1,:).*5+5,coverageResults.path(2,:).*5+5, 'y')
title('CoverageMap')
hold off

x = linspace(0, size(coverageResults.path,2), size(coverageResults.path,2));
figure()
plot(x,coverageResults.path(1,:),x,coverageResults.path(2,:),x,coverageResults.path(3,:));

figure()
surf(coverageResults.particleCoverageMap')
figure()
x = linspace(0, size(coverageResults.spreads,1), size(coverageResults.spreads,1));
plot(x,coverageResults.spreads,x,coverageResults.coverages);
