% Example 01:
% Generate a map of the environment based on real data, improve mapping
% parameters and evaluate map results
clear all
close all
clc

%% Choose the data set and load the data, also the groundtruth map, if it exists
data = 'garden03.mat';
load(data)                  % Load real data
load(['map_' data]);        % Load groundtruth map

iter = 20;
results = cell(iter,2);
for i=1:iter
%% Initialize the control unit
controlUnit = ControlUnit(polyMap,zeros(3,1));

%% Generate map estimate from odometry data
optimize.loopClosure = true;
optimize.mapping = 1;
[controlUnit,mappingResults] = controlUnit.mapping(pose,optimize);

%% Compare estimated map with groundtruth
[controlUnit,comparisonResults] = controlUnit.compare(6);

%% Add
results{i,1} = mappingResults;
results{i,2} = comparisonResults;
end

%% Evaluate Results
iter = 20;
compResult = zeros(iter,1);
for i=1:iter
    compResult(i) = results{i,2}.error;
end
idx = [];
for i=1:iter
    if isinf(compResult(i))
        idx = [idx, i];
    end
end
compResult(idx) = [];
mu = mean(compResult)
sigma = std(compResult)
errors = length(idx)
