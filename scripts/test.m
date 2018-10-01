% This a script in order to test the simulation environment.
%
% Author: Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
% Date: 01.10.2018

%% Clear everything
close all;
clear all;
clc;

%% Parameters
pose = [4; 8; 0];                   % start pose
pose_store = pose;
numParticles = [50; 500];          % TODO: add this to the config file
map_name = 'map_01.mat';         	% Choose the map which should be loaded
load(map_name); 

%% Run the Particle Filter, TODO: Add plotting as function to pf
pf = ParticleFilter(numParticles,polyMap,[pose; 0]);
figure(1)
particles = getParticlePosition(pf);
h1 = scatter(particles(1,:),particles(2,:),5,'r', 'filled');
hold on
h2 = scatter(pose(1),pose(2),30,'g', 'filled');
hold on
h3 = plot(polyMap.x,polyMap.y,'k');     % polygon
axis equal


%% Run the iteration
ctrl = RandomController();
tic
for i = 1:10000
    % Step 1: Get sensor measurements
    [sensorData] = grassSensor(pose,polyMap);
    
    % Step 2: Get control signals
    [ctrl,u] = randomControl(ctrl,sensorData);

    % Step 3: Move Robot and store positions
    [pose, motionData] = kinModel(pose, u, false);
    
    % Step 4: Particle Filter algorithm
    [odometryData] = odometryModel(pose_store, motionData);
    pf = update(pf,odometryData,sensorData);
    
    % Step 6: Print actual outcome
    figure(1)
    delete(h1)
    delete(h2)
    particles = getParticlePosition(pf);
    h1 = scatter(particles(1,:),particles(2,:),5,'r', 'filled');
    hold on
    h2 = scatter(pose(1),pose(2),30,'g', 'filled');
    drawnow   
    
    % Store Pose
    pose_store = pose;
    i
end