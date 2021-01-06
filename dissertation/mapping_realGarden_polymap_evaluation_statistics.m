% Mapping Scheme for real environments
% Evaluate for all real data

clear all
close all
clc

%% Load the data
map = 'garden03.mat';
load(map)                  % Load the real data
load(['map_' map]);        % Load groundtruth map

%% Parameter
optimize.loopClosure.flag = 1;
optimize.loopClosure.plotting.flag = 0;
optimize.loopClosure.plotting.width = 16.2/3;    % centimeter for a page site
optimize.loopClosure.plotting.height = 5.0;
optimize.mapping = 0;

mode.loopClosure = 1;       % 1: Scan alignment (ECMR Paper), 2: ICP
mode.mapping = 2;           % 1: ECMR Paper, 2: ICP as proposed in IROS Workshop Paper

plotting.flag = false;
plotting.width = 16.2/3;    % centimeter for a page site
plotting.height = 5.0;


%% Iterations
n = 20;

results = cell(n,1);
params = cell(n,1);

parfor i=1:1:n
    %% Initialize the control unit
    controlUnit = ControlUnit(polyMap,zeros(3,1));

    %% Generate map estimate from odometry data
    [controlUnit,mappingResults] = controlUnit.mapping(pose,optimize,mode,plotting);
    param{i} = mappingResults.param;

    %% Compare estimated map with groundtruth
    [controlUnit,comparisonResults] = controlUnit.compare(6);
    results{i} = comparisonResults;   
end

%% Save data
save(['results_',map],'results','param')

%% Evaluate the data
l_nh = zeros(n,1);
c_max = zeros(n,1);
gamma_1 = zeros(n,1);
gamma_2 = zeros(n,1);
error = zeros(n,1);
for i=1:1:n
    l_nh(i) = param{i}.l_nh;
    c_max(i) = param{i}.c_max;
    gamma_1(i) = param{i}.gamma(1);
    gamma_2(i) = param{i}.gamma(2);
    error(i) = results{i}.error;
end

mean(l_nh)
std(l_nh)
mean(c_max)
std(c_max)


% idx = [2 3 4 8 9 10 11 15 16 17 20];

mean(error)
std(error)

% mean(error(idx))
% std(error(idx))


