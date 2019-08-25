% This script transforms the ROS Bag data into a .mat format.
%
% Nils Rottmann
% 01.08.2018
%% Clear everything
close all;
clear all;
clc;

%% Load rosbag data and extract odometry data
bag = rosbag('Garden04.bag');
start = bag.StartTime;
stop = bag.EndTime;
bagselectOdometry = select(bag, 'Time', [start stop], 'Topic', '/odometryData');
odometry = timeseries(bagselectOdometry, 'LR', 'LL','RawR', 'RawL');
msg_odometry = readMessages(bagselectOdometry);
l_odometry = max(size(msg_odometry));

%% Evaluate Raw Odometry
dRight = 0.213;
dLeft = 0.2128;
    
nHall = 703;                        % Number of Hallsensors for one round
maxCount = 65535;                   % maximum count, 2 bytes
lTick(1) = dRight * pi / nHall;     % Length per tick for the left wheel
lTick(2) = dLeft * pi / nHall;     	% length per tick for the right wheel
treshold = 10*nHall;

% Calculate odometry data from raw data
lOdo = zeros(length(odometry.Time)-1,2);
for j=1:1:2
    for i=2:1:length(odometry.Time)
        diff = odometry.Data(i,2+j) - odometry.Data(i-1,2+j);
        if (diff > treshold)        % negative overshooting
            lOdo(i-1,j) = -(odometry.Data(i-1,2+j) + maxCount - odometry.Data(i,2+j)) * lTick(j);
        elseif (diff < -treshold)   % positive overshooting
            lOdo(i-1,j) = (odometry.Data(i,2+j) + maxCount - odometry.Data(i-1,2+j)) * lTick(j);
        else
            lOdo(i-1,j) = diff * lTick(j);
        end
    end
end

% Store odometry data in struct
odo.l_R = lOdo(:,1);       
odo.l_L = lOdo(:,2);
odo.N = length(odometry.Time)-1;

%% Evaluate Odometry Data
% Parameter
L = 0.1845;

% Evaluation
% l_R = odometry.Data(:,1);  
% l_L = odometry.Data(:,2);
l_R = odo.l_R;  % Raw Data
l_L = odo.l_L;
l_odometry = length(l_R);
pose = zeros(3,l_odometry+1);
for i=1:1:l_odometry
    ds = (l_R(i) - l_L(i))/2;
    dphi = (l_R(i) + l_L(i))/(2*L);
    pose(:,i+1) = pose(:,i) + [cos(pose(3,i)) 0; sin(pose(3,i)) 0; 0 1]*[ds; dphi];
end

%% Store odometry data
save('Garden04.mat','pose');
