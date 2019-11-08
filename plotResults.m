% close all, clear all, clc;

map = 'map_01.mat';  
load(map);
mapAbs = mapAbsolute(polyMap, 10);
times = zeros(15,2);
truecoverages = zeros(15,2);

%% Random

for i = 1:1:15
    results = strcat('planned99error_', num2str(i));
    results = strcat(results, '.mat');
    load(results);
    
    truecoverages(i,1) = sum(coverageResults.groundTruth==1)/sum(mapAbs==1);
    times(i,1) = length(coverageResults.coverages);
end

%% Planned

for i = 1:1:15
    results = strcat('planned99error90T_', num2str(i));
    results = strcat(results, '.mat');
    load(results);
    
    truecoverages(i,2) = sum(coverageResults.groundTruth==1)/sum(mapAbs==1);
    times(i,2) = length(coverageResults.coverages);
end

mean75 = mean(truecoverages(:,1));
std75 = std(truecoverages(:,1));
mean90 = mean(truecoverages(:,2));
std90 = std(truecoverages(:,2));

%% Plot
% close all;

figure
bar(75, 1, 2, 'FaceColor', '#0072BD')
hold on
b1 = bar(75, mean75+std75, 2, 'FaceColor', '#D95319')
b2 = bar(75, mean75-std75, 2, 'FaceColor', '#77AC30')
bar(90, 1, 2, 'FaceColor', '#0072BD')
bar(90, mean90+std90, 2, 'FaceColor', '#D95319')
bar(90, mean90-std90, 2, 'FaceColor', '#77AC30')
p1 = plot([74,76], [mean75,mean75],  'k', 'LineWidth', 2)
text(76.2,mean75,strcat('\leftarrow',num2str(mean75),'\pm',num2str(std75)))
plot([89,91], [mean90,mean90], 'k', 'LineWidth', 2)
text(83,mean90,strcat(num2str(mean90),'\pm',num2str(std90),'\rightarrow'))
xticks([75 90])
xticklabels({'0.75','0.9'})
legend([p1 b1],'Mittelwert','Standardabweichung','Location','south')
legend('boxoff')
xlabel('Grenzwert', 'Interpreter', 'Latex')
ylabel('Abdeckung', 'Interpreter', 'Latex')