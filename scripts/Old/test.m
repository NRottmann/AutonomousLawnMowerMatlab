% This a script in order to test the simulation environment. Please adjust
% the parameters below to your convenience
%
% Author: Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
% Date: 04.10.2018

%% Clear everything
close all;
clear all;
clc;

%% Choose Parameters
pose = [4; 8; 0];                   % start pose
pose_store = pose;
numParticles = [200; 200];          % TODO: add this to the config file
map_name = 'map_01.mat';         	% Choose the map which should be loaded
load(map_name); 

%% Initialize classes
ctrl = WallFollower();
grassSensor = GrassSensor(polyMap);
odometryModel = OdometryModel();
pf = ParticleFilter(numParticles,polyMap,[pose; 1], grassSensor, odometryModel);

%% Do the first plot
figure(1)
particles = getParticlePosition(pf);
h1 = scatter(particles(1,:),particles(2,:),5,'r', 'filled');
hold on
h2 = scatter(pose(1),pose(2),30,'g', 'filled');
hold on
h3 = plot(polyMap.x,polyMap.y,'k');     % polygon
axis equal

%% Run the iteration
odometryData.deltaR1 = 0;
odometryData.deltaR2 = 0;
tic
for i = 1:10000
    % Step 1: Get sensor measurements
    [sensorData] = measure(grassSensor,pose);
    
    % Step 2: Get control signals
    [ctrl,u] = wallFollowing(ctrl,sensorData);
    
    % Step 3: Move Robot and store positions
    [pose, motionData] = kinModel(pose, u, false);
    
    % Step 4: Particle Filter algorithm
    pf = update(pf,sensorData,motionData,pose);
    
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
end