% test something
close all
clear all
clc

%% Load
load('results_map_10.mat');

n = 10;
modes = 3;

points = 100;

%% Plot results
coverage = cell(3,1);
travDist = cell(3,1);
for j=1:modes
    coverageTmp = [];
    travDistTmp = [];
    for i=1:n
        coverageTmp = [coverageTmp, results{i}{j}.trueCoverage];
        travDistTmp = [travDistTmp, results{i}{j}.travelledDist];
    end
    coverage{j} = coverageTmp;
    travDist{j} = travDistTmp;
end

% Put all together
x = cell(3,1);
mu = cell(3,1);
sigma = cell(3,1);
for j=1:modes
    minDist = min(travDist{j}(end,:));
    x{j} = linspace(0,minDist,points);
    
    Y = zeros(points,n);
    for i=1:n
        idx = [];
        distTmp = travDist{j}(:,i);
        covTmp = coverage{j}(:,i);
        for l=2:length(distTmp)
            if (distTmp(l) == distTmp(l-1))
                idx = [idx l];
            end
        end
        distTmp(idx) = [];
        covTmp(idx) = [];
        
        Y(:,i) = interp1(distTmp,covTmp,x{j});
    end
    
    mu{j} = mean(Y,2);
    sigma{j} = std(Y,0,2);
end

%% Plot

modesLabels = {'Random','Optimal','NNCCPP'};

% Define Colors
cols = lines(modes);
cols = cols(end:-1:1,:);
% Define fontsizes
fs = 10;

width = 16.2;    % centimeter for a page site
height = 5.0;

h = figure;
set(h, 'Units','centimeters','Position', [1 1 width height])
hold on
for i=1:modes
    H = shadedErrorBar(x{i},mu{i},sigma{i}, {'-', 'color', cols(i,:), 'markerfacecolor', cols(i,:), 'linewidth', 1.5},1);
    lHnd(1,i) = H.mainLine;
end

legend(lHnd, modesLabels)
legend boxoff    
box off
set(gca,'fontsize',fs);
xlabel('Travelled Distance', 'fontsize',fs);
ylabel('Coverage', 'fontsize',fs);
axis([0 230 0 1])



