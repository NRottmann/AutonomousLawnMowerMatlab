% close all, clear all, clc;

load('random99_map5.mat');

% figure;
% subplot(1,2,1)
% plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
% hold on
% plot(coverageResults.path(1,:),coverageResults.path(2,:))
% pbaspect([1 1 1])
% set(gca,'visible','off')

waysRand = zeros(1,length(coverageResults.path)-1);
for j=2:1:length(coverageResults.path)-1
    waysRand(j) = waysRand(j-1)+norm([coverageResults.path(1,j)-coverageResults.path(1,j-1),coverageResults.path(2,j)-coverageResults.path(2,j-1)]);
end

figure;
plot(waysRand, coverageResults.coverages, 'LineWidth', 2)
hold on

load('planned99error90T_map5.mat');

% subplot(1,2,2)
% plot(coverageResults.polyMap.x,coverageResults.polyMap.y)
% hold on
% plot(coverageResults.path(1,:),coverageResults.path(2,:))
% pbaspect([1 1 1])
% set(gca,'visible','off')

waysPlan = zeros(1,length(coverageResults.path));
for j=2:1:length(coverageResults.path)
    waysPlan(j) = waysPlan(j-1)+norm([coverageResults.path(1,j)-coverageResults.path(1,j-1),coverageResults.path(2,j)-coverageResults.path(2,j-1)]);
end
mapAbs = mapAbsolute(coverageResults.polyMap, 10);
truecov = path2coverage(coverageResults.path, mapAbs, coverageResults.polyMap, length(coverageResults.path));

plot(waysPlan, truecov, 'LineWidth', 2)
xlabel('gefahrene Strecke in Meter', 'Interpreter', 'Latex')
ylabel('tats\"achliche Abdeckung', 'Interpreter', 'Latex')
legend({'Randomwalk','Pfadplaner'},'Location','southeast', 'Interpreter', 'Latex')
legend('boxoff')

maxcov = max(truecov);


% time = length(coverageResults.coverages)/20;
% time = time/60
% 
% figure;
% surf(coverageResults.neuralActivity')