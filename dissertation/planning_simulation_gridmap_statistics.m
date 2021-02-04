% Script for performing statistics for the CCPP algorithm
clear all
close all
clc

% Parameters
M = 6;                  % Number of maps
N = 20;                 % Number of samples for each map
reqCoverage = 1.1;      % Coverage in percentage
maxTime = 10800;        % maximum time in seconds (10800s = 3h)

results = cell(N,M);

% Go over all maps
for j=1:M
    
    map = strcat('map_',num2str(j),'.mat');
    load(map);
    pose = generateStartPose(polyMap);

    parfor i=1:N

        modeResults = cell(3,1);

        %% Mode 1
        % Initialize the control unit
        controlUnit = ControlUnit(gridMap,pose);

        % Complete Coverage with particle filter localization
        [~,modeResults{1}] = controlUnit.completeCoverage(reqCoverage,maxTime,1); 


        %% Mode 2
        % Initialize the control unit
        controlUnit = ControlUnit(gridMap,pose);

        % Complete Coverage with particle filter localization
        [~,modeResults{2}] = controlUnit.completeCoverage(reqCoverage,maxTime,2); 


        %% Mode 3
        % Initialize the control unit
        controlUnit = ControlUnit(gridMap,pose);

        % Complete Coverage with particle filter localization
        [~,modeResults{3}] = controlUnit.completeCoverage(reqCoverage,maxTime,3); 

        
        %% Store results
        results{i,j} = modeResults;

    end
    
end

%% Get all parameters and save the results
[config] = getCompleteConfig();
save('coverageStatistics','results','config')


