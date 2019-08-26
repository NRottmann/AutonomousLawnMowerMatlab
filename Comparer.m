clear all, close all, clc;

results = 'random20.mat';

load(results);
mapAbs = mapAbsolute(coverageResults.polyMap, 5);

p = immse(coverageResults.groundTruth, coverageResults.particleCoverageMap)
c = immse(coverageResults.groundTruth, coverageResults.coverageMap)
tmp = coverageResults.groundTruth;
tmp(mapAbs==0) = 0;
coverage = sum(tmp==1)/sum(mapAbs==1)

figure()
title('')
surf(coverageResults.particleCoverageMap')
grid on
xlabel('x in Zellen')
ylabel('y in Zellen')
colorbar
pbaspect([1 1 1])

figure()
surf(coverageResults.coverageMap')
grid on
xlabel('x in Zellen')
ylabel('y in Zellen')
colorbar
pbaspect([1 1 1])