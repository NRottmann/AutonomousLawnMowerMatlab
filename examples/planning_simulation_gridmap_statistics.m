% Script for performing the CCPP algorithm

clear all
close all
clc

%% Define map and starting pose
mapName = 'map_10';
map = strcat(mapName,'.mat');
load(map);
pose = [2.05; 0.05; 0];


%% Iterations
n = 10;

reqCoverage = 1.1;           % Coverage in percentage
maxTime = 3000;              % maximum time in seconds

results = cell(n,1);

parfor i=1:n
    
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
    
   
    
    results{i} = modeResults;
    
    disp(i)
    
end

%% Save the results
saveName = strcat(mapName,'.mat');
save(saveName,'results')


%% Plot some results
% figure()
% subplot(1,2,1)
% plot(polyMap.x,polyMap.y)
% hold on
% plot(coverageResults.path(1,:),coverageResults.path(2,:))
% subplot(1,2,2)
% plot(polyMap.x,polyMap.y)
% hold on
% plot(coverageResults.estPath(1,:),coverageResults.estPath(2,:))
% hold off
% figure()
% surf(coverageResults.groundTruth');
% title('groundTruth');
% figure()
% surf(coverageResults.particleCoverageMap');
% title('ParticleCoverageMap');
% figure()
% surf(coverageResults.coverageMap');
% title('CoverageMap');
% figure()
% particle = coverageResults.particleCoverageMap;
% particle(particle < controlUnit.Threshhold) = 0;
% particle(particle >= controlUnit.Threshhold) = 1;
% surf(particle');
% title('ParticleCoverageMapThreshhold');
% figure()
% coverage = coverageResults.coverageMap;
% coverage(coverage < controlUnit.Threshhold) = 0;
% coverage(coverage >= controlUnit.Threshhold) = 1;
% surf(coverage');
% title('CoverageMapThreshhold');
% if mode == 2 || mode == 4
%     figure()
%     surf(coverageResults.neuralActivity');
%     title('Neural Activity');
%     figure()
%     surf(coverageResults.externalInput');
%     title('External Input');
% end
% figure()
% surf(coverageResults.coverageMap')
% grid on
% xlabel('x in Zellen')
% ylabel('y in Zellen')
% title('Abdeckungskarte')
% colorbar
% pbaspect([1 1 1])
% figure()
% surf(coverageResults.particleCoverageMap')
% grid on
% xlabel('x in Zellen')
% ylabel('y in Zellen')
% title('Abdeckungskarte')
% colorbar
% pbaspect([1 1 1])
