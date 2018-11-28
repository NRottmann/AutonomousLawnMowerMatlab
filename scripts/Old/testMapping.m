% This a script in order to test the simulation environment. Please adjust
% the parameters below to your convenience
%
% Author: Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
% Date: 05.10.2018

%% Clear everything
close all;
clear all;
clc;

%% Choose Parameters
pose = [0; 0; 0];                   % start pose
pose_store = pose;
pose_est = pose;
map_name = 'map_01.mat';         	% Choose the map which should be loaded
load(map_name); 

%% Initialize classes
ctrl = WallFollower();
grassSensor = GrassSensor(polyMap);
odometryModel = OdometryModel();

%% Run the iteration
N = 20000;
odoData = zeros(2,N);
for i = 1:N
    % Step 1: Get sensor measurements
    [sensorData] = measure(grassSensor,pose);
    
    % Step 2: Get control signals
    [ctrl,u] = wallFollowing(ctrl,sensorData);
    
    % Step 3: Move Robot and store positions
    [pose, motionData] = kinModel(pose, u, false);
    
    % Get odometry data
    odometryModel = odometryModel.odometryData(pose_est, motionData);
    [pose_est] = odometryModel.odometryPose(pose_est, true);
    
    % Store odo data
    odoData(:,i) = pose_est(1:2);
    
    % Store Pose
    pose_store = pose;
end

%% Plot odoData
figure(1)
plot(odoData(1,:),odoData(2,:))

%% Generate map
mapping = MappingOdometry(odoData);
[polyMap,fittingPairs,DP,DP_opt] = map(mapping);

%% Plot polyMap
figure(2)
plot(polyMap.x,polyMap.y)