close all, clear all, clc;

results = 'random_1.mat'; 
load(results);

figure()
subplot(4,3,1)
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.path25(1,:),coverageResults.path25(2,:))
subplot(4,3,4)
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.path50(1,:),coverageResults.path50(2,:))
subplot(4,3,7)
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.path75(1,:),coverageResults.path75(2,:))
subplot(4,3,10)
plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
hold on
plot(coverageResults.path(1,:),coverageResults.path(2,:))

subplot(4,3,2)
surf(coverageResults.coverageMap25')
colorbar
subplot(4,3,5)
surf(coverageResults.coverageMap50')
colorbar
subplot(4,3,8)
surf(coverageResults.coverageMap75')
colorbar
subplot(4,3,11)
surf(coverageResults.coverageMap')
colorbar

subplot(4,3,3)
surf(coverageResults.groundTruth25')
subplot(4,3,6)
surf(coverageResults.groundTruth50')
subplot(4,3,9)
surf(coverageResults.groundTruth75')
subplot(4,3,12)
surf(coverageResults.groundTruth')