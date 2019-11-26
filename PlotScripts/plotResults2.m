close all, clear all, clc;

map = 'map_01.mat';  
load(map);
mapAbs = mapAbsolute(polyMap, 10);
maxRand = 471115;
maxPlan = 352168;
times = zeros(15,2);
truecoverages = zeros(15,1);

%% Random

coveragesRand = 0.99*ones(15,maxRand);
waysRand = zeros(1,maxRand);
for i = 1:1:15
    results = strcat('random99_', num2str(i));
    results = strcat(results, '.mat');
    load(results);
    
    waysRand_tmp = zeros(1,maxRand);
    times(i,1) = length(coverageResults.coverages);
    coveragesRand(i,1:length(coverageResults.coverages)) = coverageResults.coverages;
    for j=2:1:length(coverageResults.path)-1
        waysRand_tmp(j) = waysRand_tmp(j-1)+norm([coverageResults.path(1,j)-coverageResults.path(1,j-1),coverageResults.path(2,j)-coverageResults.path(2,j-1)]);
    end
    waysRand_tmp(length(coverageResults.path):end) = max(waysRand_tmp);
%     for j = length(coverageResults.path):1:maxRand
%         waysRand_tmp(j) = max(waysRand_tmp);
%     end
    waysRand = waysRand+(waysRand_tmp/15);
end
meanCovRand = mean(coveragesRand);
stdCovRand = std(coveragesRand);

%% Planned

% coveragesPlan = 0.99*ones(15,maxPlan);
trueCovs = zeros(15,maxPlan+1);
waysPlan = zeros(1,maxPlan+1);

for i = 1:1:15
    results = strcat('planned99error90T_', num2str(i));
    results = strcat(results, '.mat');
    load(results);
    lang(i) = length(coverageResults.coverages);
    truecoverages(i) = sum(coverageResults.groundTruth==1)/sum(mapAbs==1);
end
max(truecoverages)

for i = 1:1:15
    disp(i);
    results = strcat('planned99error90T_', num2str(i));
    results = strcat(results, '.mat');
    load(results);
    
    waysPlan_tmp = zeros(1,maxPlan+1);
    truecoverages(i) = sum(coverageResults.groundTruth==1)/sum(mapAbs==1);
    times(i,2) = length(coverageResults.coverages);
%     coveragesPlan(i,1:length(coverageResults.coverages)) = coverageResults.coverages;
    for j=2:1:length(coverageResults.path)
        waysPlan_tmp(j) = waysPlan_tmp(j-1)+norm([coverageResults.path(1,j)-coverageResults.path(1,j-1),coverageResults.path(2,j)-coverageResults.path(2,j-1)]);
    end
    waysPlan_tmp(length(coverageResults.path):end) = max(waysPlan_tmp);
%     for j = length(coverageResults.path):1:maxplan+1
%         waysPlan_tmp(j) = max(waysPlan_tmp);
%     end
%     waysPlan_tmp(waysPlan_tmp == 0) = max(waysPlan_tmp);
    waysPlan = waysPlan+(waysPlan_tmp/15);
    trueCov = path2coverage(coverageResults.path,mapAbs,polyMap,maxPlan+1);
    trueCovs(i,:) = trueCov;
end
meanCovPlan = mean(trueCovs);
stdCovPlan = std(trueCovs);

%% Plot
disp('plotting')
close all;

Xrand = linspace(0, maxRand/20, maxRand);
Yrand = [meanCovRand+1*stdCovRand; meanCovRand-1*stdCovRand];
Xplan = linspace(0, maxPlan/20, maxPlan+1);
Yplan = [meanCovPlan+1*stdCovPlan; meanCovPlan-1*stdCovPlan];

figure
plot(waysRand, meanCovRand)
hold on
plot(waysPlan, meanCovPlan)
hold off

h=figure
s = shadedErrorBar(waysRand, Yrand,{@mean,@std},'lineprops','-r','patchSaturation',0.2);
s.mainLine.LineWidth = 2;
hold on
s = shadedErrorBar(waysPlan, Yplan,{@mean,@std},'lineprops','-b','patchSaturation',0.2);
s.mainLine.LineWidth = 2;
xlabel('gefahrene Strecke in Meter', 'Interpreter', 'Latex')
ylabel('tats\"achliche Abdeckung', 'Interpreter', 'Latex')
legend({'Randomwalk','Pfadplaner'},'Location','southeast', 'Interpreter', 'Latex')
legend('boxoff')
hold off
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'map1pdf','-dpdf','-r0')

% figure;
% plot(meanCovPlan)
% hold on
% plot(meanCovRand)
% hold off
% 
% figure
% s = shadedErrorBar(Xrand, Yrand,{@mean,@std},'lineprops','-r','patchSaturation',0.2);
% s.mainLine.LineWidth = 2;
% hold on
% s = shadedErrorBar(Xplan, Yplan,{@mean,@std},'lineprops','-b','patchSaturation',0.2);
% s.mainLine.LineWidth = 2;
% xlabel('gefahrene Zeit in Sekunden', 'Interpreter', 'Latex')
% ylabel('tats\"achliche Abdeckung', 'Interpreter', 'Latex')
% legend({'Randomwalk','Pfadplaner'},'Location','southeast', 'Interpreter', 'Latex')
% legend('boxoff')
% hold off