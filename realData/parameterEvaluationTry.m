% This script analyzes the odometry received from the robot and calculates
% based on the true poses (optiTrack measurements) the robot specific
% parameters
%
% Authot:   Nils Rottmann
% E-Mail:   Nils.Rottmann@rob.uni-luebeck.de
%
% Support:  Rico Klinckenberg
%
% TODO: Clean this up!

close all
clear all
clc

%% Flags and data set
plotFlag = false;        % If true, we plot results
delayEstFlag = false;   % If true, we estimate the best time delay
robotParamFlag = false;  % If true, we estimate the best robot parameters
modelParamFlag = true;  % If true, we estimate the best model parameters

numDataSet = 1;         % Number of the data set   


%% Load the data
% Load files as .mat if thex exists
if exist(['parameterEvaluation',num2str(numDataSet),'.mat'],'file')
    load(['parameterEvaluation',num2str(numDataSet),'.mat']);
    disp('Loaded .mat files!')
% If not, load the bag data
elseif (exist(['parameterEvaluation',num2str(numDataSet),'.bag'],'file') && exist(['parameterEvaluation',num2str(numDataSet),'.csv'],'file'))
    [bagData, optiTrackData] = loadData(['parameterEvaluation',num2str(numDataSet),'.bag'],['parameterEvaluation',num2str(numDataSet),'.csv']);
    disp('Loaded .bag and .csv files! Preprocessing required!')
    [optiTrackData] = removePeaks(optiTrackData);
    disp('Removed peaks from OptiTrack data!')
    [optiTrack] = convertOptiTrackData(optiTrackData);
    disp('Extract Pose Information!')
    [odometry,velocity] = processBagData(bagData);
    disp('Get odometry ticks from the bag data!')
    save(['realData/parameterEvaluation',num2str(numDataSet),'.mat'],'optiTrack','odometry','velocity')
    disp('Saved Data to .mat file!')
else
    errorOdo(['File parameterEvaluation',num2str(numDataSet),'.bag or parameterEvaluation',num2str(numDataSet),'.csv does not exist!'])
end

%% Parameter, or in case of optimization processes we assume this are the 
% intial parameters
param = [-6.85; 0.2148; 0.2145; 0.1828];        % [tDelay,dR,DL,L]
aOdo = [0.01; 0.01; 0.01; 0.01];                % Start parameters for optimization
aVel = [0.01; 0.01; 0.01; 0.01; 0.01; 0.01]; 

%% Find a good time delay between optiTrack data and odometry data
if delayEstFlag
    [param] = optimizeRobotParams(odometry,optiTrack,param,1);
    disp('Found time delay!')
end

%% Find convenient robot parameters
if robotParamFlag
    [param] = optimizeRobotParams(odometry,optiTrack,param,2);
    disp('Optimized robot specific parameter!')
end
    
%% Adjust robot data and optiTrack data such that they are fit together
% Get way driven by the wheels from the ticks
[odometry] = generateOdomLength(odometry,param(2:3),param(1));
velocity.Time = velocity.Time - velocity.Time(1) - (-6.5);
% Interpolate OptiTrackData such that times fit together
[optiTrackInterOdo] = interpolatingOptitrack(optiTrack,odometry);
[optiTrackInterVel] = interpolatingOptitrack(optiTrack,velocity);
% Calculate odometry pose estimates
[odometry] = getOdometryPose(optiTrackInterOdo, odometry, param(4));
[velocity] = getVelocityPose(optiTrackInterVel, velocity);
disp('Fit data sets together!')

%% Allocate odometry data to the optiTrack measurements
% TODO: Improve formulasr such that dN is not longer required
% [selection] = allocateOdometryData(1,odometry, optiTrackInterOdo, param(4));
% disp('Odometry data allocated!')
% 
% %% Get the odometry errors
% [errorOdo] = odometryError(selection);
% disp('Calculated odometry errors!')

%% Get the velocity errors
[errorVel] = velocityError(optiTrackInterVel,velocity);
% maxN = 10000;
% errorVel.Error = errorVel.Error(:,1:maxN);
% errorVel.Ctrl = errorVel.Ctrl(:,1:maxN);


%% Debugging
M = length(errorVel.Error(:,1));
N = length(errorVel.Error(1,:));
x = zeros(N*M,1);
for j=1:1:M
    for i=1:1:N
        x(i+(j-1)*N) = errorVel.Error(j,i);
    end
end
n = 500;
t = linspace(0.001,10,n);
for i=1:1:n
    a = ones(6,1) * t(i);
    [L(i)] = getLogLikelihood(x,errorVel,a,2);
    i
end
plot(t,L)

%% Estimate the model parameters
% if modelParamFlag
% %     [aOdo] = estimateParameter(errorOdo,aOdo,1);
%     [aVel] = estimateParameter(errorVel,aVel,2);
% end

%% Plots
if plotFlag == true
    figure(1);
    subplot(1,2,1)
    plot(optiTrack.Pose(:,1),optiTrack.Pose(:,2))
    hold on
    xlabel('$$x$$ in meter','Interpreter','latex')
    ylabel('$$y$$ in meter','Interpreter','latex')
    title('Motion Capture','Interpreter','latex')
    box off
    set(gcf,'PaperPositionMode','auto')
    subplot(1,2,2)
    plot(odometry.Pose(:,1),odometry.Pose(:,2))
    hold on
    xlabel('$$x$$ in meter','Interpreter','latex')
    ylabel('$$y$$ in meter','Interpreter','latex')
    title('Odometry','Interpreter','latex')
    box off
    set(gcf,'PaperPositionMode','auto')
end
  
%% General Functions
function [bag,optiTrackData] = loadData(bag,optiTrack)
    % Loads bag file and optiTrack file
    % data from ROS from the lawn mower
    bag = rosbag(bag);
    % Data from the OptiTrack System, Frame,Time,X,Y,Z,W,X,Y,Z,
    optiTrackData = csvread(optiTrack,7,0);
end

function [optiTrackData] = removePeaks(optiTrackData)
    % This function removes the peaks from the OptiTrack data which are
    % physically not possible.
    % Removing false OptiTrack Data (spikes) which are physically not possible
    %
    % TODO: This can be improved!
    
    deviation = 0.01;
    
    for j=1:1:20
        i = 2;
        while i < length(optiTrackData(:,2))
            if abs(optiTrackData(i,7)) > abs(optiTrackData(i-1,7))+deviation || optiTrackData(i,7) < optiTrackData(i-1,7)-deviation
                optiTrackData(i,:) = [];
            elseif abs(optiTrackData(i,8)) > abs(optiTrackData(i-1,8))+deviation || optiTrackData(i,8) < optiTrackData(i-1,8)-deviation
                optiTrackData(i,:) = [];
            elseif abs(optiTrackData(i,9)) > abs(optiTrackData(i-1,9))+deviation || optiTrackData(i,9) < optiTrackData(i-1,9)-deviation
                optiTrackData(i,:) = [];
            end
            i = i + 1;
        end
    end
end

function [optiTrack] = convertOptiTrackData(optiTrackData)
    % This functions generates the required data (x,y,phi) from the raw
    % OptiTrack data, since we only assume 2D movement
    %
    % Thereby; x_raw = x, z_raw = -z, phi = phi (rotation around y-axis)
    
    % Get OptiTrack timestamps
    time = optiTrackData(:,2);
    N = length(time);
    
    % Get Position Data
    optiTrackPose = optiTrackData(:,[7 9]);
    
    % Generate angles
    phiTmp = zeros(N,1);
    for i=1:1:N
        % Convert quaternion to rotation matrix, q = [w x y z]
        R = quat2rotm([optiTrackData(i,6) optiTrackData(i,3:5)]);
        % Calculcate orientation around y-axis
        phiTmp(i,1) = atan2(-R(3,1),R(1,1));
    end
    optiTrackPose = [optiTrackPose phiTmp];
    
    % Transform z-positions into x-y-frame
    optiTrackPose(:,2) = -optiTrackPose(:,2);
    
    % Put data together
    optiTrack.Pose = optiTrackPose;
    optiTrack.Time = time;
    optiTrack.N = N;
end

function [odometry,velocity] = processBagData(bag)

    % Parameters, TODO: Put this into the config
    maxCount = 65535;       % Max number of counts, 2 Bytes
    nHall = 703;            % Ticks per wheel rotation
    treshold = 10*nHall;    % Treshold to determine overflow of register

    % Load Odometry data
    start = bag.StartTime;
    stop = bag.EndTime;
    
    % Select the odometry data
    bagselectOdo = select(bag, 'Time', [start stop], 'Topic', '/odometryData');
    bagselectVel = select(bag, 'Time', [start stop], 'Topic', '/controlData');
    
    % Get the raw odometry data (the actual ticks number at time t)
    odo = timeseries(bagselectOdo, 'RawR', 'RawL');
    vel = timeseries(bagselectVel, 'V', 'W');
    
    % Calculate Odometry Data ticks per time stamp
    ticks = zeros(length(odo.Time)-1,2);
    for j=1:1:2
        for i=2:1:length(odo.Time)
            diff = odo.Data(i,j) - odo.Data(i-1,j);
             if (diff > treshold)        % negative overshooting
                 ticks(i-1,j) = -(odo.Data(i-1,j) + maxCount - odo.Data(i,j));
             elseif (diff < -treshold)   % positive overshooting
                 ticks(i-1,j) = (odo.Data(i,j) + maxCount - odo.Data(i-1,j));
             else
                 ticks(i-1,j) = diff;
             end
        end
    end
    odometry.Ticks = ticks;
    odometry.Time = odo.Time;
    
    % Do not use first positions where there is no movement. Otherwise 
    % interpolating optitrack would cause problems.
    flag = false;
    t = 1;
    for i=1:1:length(vel.Time)
        if vel.Data(i,1) ~= 0 ||  vel.Data(i,2) ~= 0
            flag = true;
        end
        if flag
         	velocity.Time(t) = vel.Time(i);
            velocity.Ctrl(t,1:2) = vel.Data(i,:);
            t = t + 1;
        end
    end
end

function [odometry] = generateOdomLength(odometry,diameter,tDelay)
    % Generates the length travelled by each wheel based on the given wheel
    % diameters
    %
    % diameter:     [dR, DL]
    
    % TODO: Put this into the config
    nHall = 703;
    
    lTick(1) = diameter(1) * pi / nHall;     % Length per tick for the right wheel
    lTick(2) = diameter(2) * pi / nHall;     % length per tick for the left wheel
 
    % Do not use first positions where there is no movement. Otherwise 
    % interpolating optitrack would cause problems.
    flag = false;
    t=1;
    for i=1:1:length(odometry.Ticks(:,1))
        lR = odometry.Ticks(i,1) * lTick(1); lL = odometry.Ticks(i,2) * lTick(2);
        if lR ~= 0 && lL ~= 0
            flag = true;
        end
        if flag
            odometry.Data(t,1) = lR;
            odometry.Data(t,2) = lL;
            timeTmp(t) = odometry.Time(i);
            t = t+1;
        end
    end

    odometry.N = length(timeTmp)-1;
    
    % Adjust odometry time accordingly to the optiTrack System by hand
    odometry.Time = timeTmp(2:end) - timeTmp(2) - tDelay;
end

function [optiTrackInter] = interpolatingOptitrack(optiTrack,data)
    % Interpolating the optiTrack Data
    % Getting interpolated OptiTrack positions based on Odometry Positions
    optiTrackInter(:,1) = interp1(optiTrack.Time,optiTrack.Pose(:,1),data.Time);
    optiTrackInter(:,2) = interp1(optiTrack.Time,optiTrack.Pose(:,2),data.Time);
    optiTrackInter(:,3) = interp1(optiTrack.Time,optiTrack.Pose(:,3),data.Time);

    % Because OptiTrack was shut down before Odometry, the last positions do not
    % exist. Using last known position for these.
    for i=2:1:length(optiTrackInter(:,1))
       if isnan(optiTrackInter(i,1))
            optiTrackInter(i,1) = optiTrackInter(i-1,1);
       end
       if isnan(optiTrackInter(i,2))
          optiTrackInter(i,2) = optiTrackInter(i-1,2); 
       end
       if isnan(optiTrackInter(i,3))
           optiTrackInter(i,3) = optiTrackInter(i-1,3);
       end
    end
end

function [odometry] = getOdometryPose(optiTrackInter, odometry, L)
    % This function uses the odometry model to calculate the pose
    % estimations
    
    % First pose aligned with first optiTrack pose
    odometry.Pose = zeros(odometry.N,3);
    odometry.Pose(1,:) = optiTrackInter(1,:);
    
    % Pose Estimation
    for i=2:1:odometry.N
        ds = (odometry.Data(i-1,1) - odometry.Data(i-1,2))/2;
        dphi = (odometry.Data(i-1,1) + odometry.Data(i-1,2))/(2*L);
        odometry.Pose(i,:) = odometry.Pose(i-1,:) ...
            + [cos(odometry.Pose(i-1,3))*ds, sin(odometry.Pose(i-1,3))*ds, dphi];
    end
    odometry.Pose(:,3) = wrapToPi(odometry.Pose(:,3));
end

function [velocity] = getVelocityPose(optiTrackInterVel, velocity)
    % This function uses the odometry model to calculate the pose
    % estimations
    
    % First pose aligned with first optiTrack pose
    N = length(velocity.Time);
    velocity.Pose = zeros(N,3);
    velocity.Pose(1,:) = optiTrackInterVel(1,:);
    
    % Pose Estimation
    for i=1:1:N-1
        dt = velocity.Time(i+1) - velocity.Time(i);
        if abs(velocity.Ctrl(i,2)) > 10^(-3)
            phi = velocity.Pose(i,3) + velocity.Ctrl(i,2) * dt;
            vw = (velocity.Ctrl(i,1)/velocity.Ctrl(i,2));
            x = velocity.Pose(i,1) + vw * (sin(phi) - sin(velocity.Pose(i,3)));
            y = velocity.Pose(i,2) + vw * (cos(velocity.Pose(i,3)) - cos(phi));
            velocity.Pose(i+1,:) = [x,y,phi];
        else
            phi = velocity.Pose(i,3);
            x =  velocity.Pose(i,1) + velocity.Ctrl(i,1)* cos(phi) * dt;
            y =  velocity.Pose(i,2) + velocity.Ctrl(i,1)* sin(phi) * dt;
            velocity.Pose(i+1,:) = [x,y,phi];
        end
    end
    velocity.Pose(:,3) = wrapToPi(velocity.Pose(:,3));
end

function [selection] = allocateOdometryData(dN,odometry, optiTrackInter, L)
    % Allocates the odometry data to the optiTrack data over dN steps
    % TODO: This step is not necessarily required! Better to include this
    % into the error estimation
    
    % Setting up needed variables
    selection.Odometry.Pose = zeros(round(odometry.N/dN),3);
    selection.OptiTrack.Pose = zeros(round(odometry.N/dN),3);
    selection.Time = zeros(round(odometry.N/dN),1);
    
    % Allocate first poses
    selection.Odometry.Pose(1,:) = optiTrackInter(1,:);
    selection.OptiTrack.Pose(1,:) = optiTrackInter(1,:);
    selection.Time(1) = odometry.Time(1);
    
    % Calculating allocated positions
    k = 2;
    for i=1:dN:(odometry.N-dN)
        poseTmp = optiTrackInter(i,:);
        for j=1:1:dN
           ii = i+j;
           ds = (odometry.Data(ii,1) - odometry.Data(ii,2))/2;
           dphi = (odometry.Data(ii,1) + odometry.Data(ii,2))/(2*L);
           poseTmp = poseTmp + [cos(poseTmp(3))*ds, sin(poseTmp(3))*ds, dphi];
        end
        % using last poseTmp for final allocated position
        selection.Odometry.Pose(k,:) = poseTmp;
        selection.OptiTrack.Pose(k,:) = optiTrackInter(i+dN,:);
        selection.Time(k,1) = odometry.Time(i+dN);
        k=k+1;
    end
end

function [error] = odometryError(selection)
    % Calculates the erros between optiTrack measurements and odometry
    % measurement based on the presented odometry model
    
    % Singularity Treshold
    singTresh = 10^(-3);
    
    % Setting up variables
    error.N = length(selection.Odometry.Pose(:,1)) - 1;
    error.Delta.OptiTrack = zeros(3,error.N);
    error.Delta.Odometry = zeros(3,error.N);
    
    % Calculating delta values for optiTrack and odometry
    for i=1:1:error.N   
        % Tranlational Movement
        error.Delta.OptiTrack(2,i) = sqrt((selection.OptiTrack.Pose(i+1,2) - selection.OptiTrack.Pose(i,2))^2 ...
            + (selection.OptiTrack.Pose(i+1,1)-selection.OptiTrack.Pose(i,1))^2);
        error.Delta.Odometry(2,i) = sqrt((selection.Odometry.Pose(i+1,2) - selection.OptiTrack.Pose(i,2))^2 ...
            + (selection.Odometry.Pose(i+1,1)-selection.OptiTrack.Pose(i,1))^2);
        % Initial Rotation
        % Check for singularity, happens for the rotation if we merely move
        if abs(error.Delta.OptiTrack(2,i)) < singTresh || abs(error.Delta.Odometry(2,i)) < singTresh
            error.Delta.OptiTrack(1,i) = 0.5 * selection.OptiTrack.Pose(i+1,3) - selection.OptiTrack.Pose(i,3);
            error.Delta.Odometry(1,i) = 0.5 * selection.Odometry.Pose(i+1,3) - selection.Odometry.Pose(i,3);
        else    
        % No Singularity
            error.Delta.OptiTrack(1,i) = atan2(selection.OptiTrack.Pose(i+1,2) - selection.OptiTrack.Pose(i,2), ...
                    selection.OptiTrack.Pose(i+1,1) - selection.OptiTrack.Pose(i,1)) - selection.OptiTrack.Pose(i,3);
            error.Delta.Odometry(1,i) = atan2(selection.Odometry.Pose(i+1,2) - selection.OptiTrack.Pose(i,2), ...
                    selection.Odometry.Pose(i+1,1) - selection.OptiTrack.Pose(i,1)) - selection.OptiTrack.Pose(i,3);
        end
        % Final Rotation
        error.Delta.OptiTrack(3,i) = selection.OptiTrack.Pose(i+1,3) - selection.OptiTrack.Pose(i,3) ...
            - error.Delta.OptiTrack(1,i);
        error.Delta.Odometry(3,i) = selection.Odometry.Pose(i+1,3) - selection.OptiTrack.Pose(i,3) ...
            - error.Delta.Odometry(1,i);
    end

    % Calculate the errors
    % If odometry is zero, we assume no movement has been occured. If we do
    % not this assumptions the noise of the optiTrack system will destroy
    % our results
    IDX{1} = find(abs(error.Delta.Odometry(1,:)) < 10^(-2));
    IDX{2} = find(abs(error.Delta.Odometry(2,:)) < 10^(-3));
    IDX{3} = find(abs(error.Delta.Odometry(3,:)) < 10^(-2));
    
    IDX{4} = find(abs(error.Delta.OptiTrack(1,:)) > 1);
    IDX{5} = find(abs(error.Delta.OptiTrack(2,:)) > 1);
    IDX{6} = find(abs(error.Delta.OptiTrack(3,:)) > 1);
    
    idx = getIndices(IDX,error.N);   
    
    error.Delta.OptiTrack(:,idx) = [];
    error.Delta.Odometry(:,idx) = []; 

    error.Error(1,:) = error.Delta.OptiTrack(1,:) - error.Delta.Odometry(1,:);
    error.Error(2,:) = error.Delta.OptiTrack(2,:) - error.Delta.Odometry(2,:);
    error.Error(3,:) = error.Delta.OptiTrack(3,:) - error.Delta.Odometry(3,:);
end

function [error] = velocityError(optiTrackInterVel,velocity)
    % Calculates the erros between optiTrack measurements and control
    % signals based on the presented odometry model
    
    % Singularity Treshold
    maxW = 1;
    singTresh = 10^(-3);
    
    % Setting up variables
    error.N = length(optiTrackInterVel(:,1)) - 1;
    
    % Calculating delta values for optiTrack and odometry
    idx = [];
    for i=1:1:error.N
        dt = velocity.Time(i+1) - velocity.Time(i);
        x0 = optiTrackInterVel(i,1);
        x1 = optiTrackInterVel(i+1,1);
        y0 = optiTrackInterVel(i,2);
        y1 = optiTrackInterVel(i+1,2);
        phi = optiTrackInterVel(i,3);
        % The robot moves along a circle
        if (optiTrackInterVel(i+1,3) - optiTrackInterVel(i,3)) < singTresh
            w = 0;
            v = norm(optiTrackInterVel(i+1,1:2) - optiTrackInterVel(i,1:2))/dt;
            g = 0;
        else
            mu = 0.5 * (((x0 - x1) * cos(phi) + (y0 - y1)*sin(phi)) ...
                        / ((y0 - y1)*cos(phi) - (x0 - x1)*sin(phi)));
            x_ = 0.5*(x0+x1) + mu*(y0-y1);
            y_ = 0.5*(y0+y1) + mu*(x1-x0);
            r = sqrt((x0-x_)^2 + (y0-y_)^2);
            dphi = atan2(y1-y_,x1-x_) - atan2(y0-y_,x0-x_);
            w = dphi / dt;
            if abs(w) > maxW       % unrealistic behavior (atan2 has failed)
                dphi = optiTrackInterVel(i+1,3) - optiTrackInterVel(i,3);
                w = dphi / dt;
            end
            dDist = r * dphi;
            v = dDist / dt;
            g = ((optiTrackInterVel(i+1,3) - optiTrackInterVel(i,3)) / dt) - w;
        end
        error.Error(1,i) = velocity.Ctrl(i,1) - v;
        error.Error(2,i) = velocity.Ctrl(i,2) - w;
        error.Error(3,i) = g;
        error.Ctrl(1,i) = v;
        error.Ctrl(2,i) = w;
        % excluding unrealistic behaviour and parts where no movement
        % occurs, since then the error from the optiTrack system would be
        % to noisy
        if (abs(v) < 0.1 ||  abs(w) < 0.1) || abs(v) > 0.3 ||  abs(w) > 0.6
            idx = [idx, i];
        end
    end
    error.Error(:,idx) = [];
    error.Ctrl(:,idx) = [];
end
function idx = getIndices(IDX,N)   
    idx = [];
    M = max(size(IDX));
    for i=1:1:N
        found = false;
        for l=1:1:M
            for j=1:1:length(IDX{l})
                if IDX{l}(j) == i
                    found = true;
                end
            end
        end
        if found
            idx = [idx i];
        end
    end
end

%% Optimization functions for the model parameters
function [a] = estimateParameter(error,a,mode)
    % This function uses the log likelihood method to estimate a convenient
    % set of parameters for the odometry model
    %
    % mode: 1: Odometry estimation
    %       2: velocity estimation
   
    % Generate the error vector
    M = length(error.Error(:,1));
    N = length(error.Error(1,:));
    x = zeros(N*M,1);
    for j=1:1:M
        for i=1:1:N
            x(i+(j-1)*N) = error.Error(j,i);
        end
    end
    
    % Get the covariance matrix and calculate the current log likelihood
    L = getLogLikelihood(x,error,a,mode);
    diff = inf;
    if mode == 1
        alpha = 10^(-8);
    elseif mode == 2
        alpha = 10^(-8);
    end
    c = 0;
    
    while diff > 10^(-9) && c < 10
        dL = getJacLogLikelihood(x,error,a,mode);
        a = a + alpha * dL;
        L_new = getLogLikelihood(x,error,a,mode);
        diff = L_new - L
        L = L_new;
        c = c + 1;
        disp(L)
    end    
end
function [L] = getLogLikelihood(x,error,a,mode)
    % Calculates the log likelihood given the measurements and the
    % covariance matrix assuming a zero mean gaussian distribution
    N = length(x);
    if mode == 1
        c = generateCovarianceOdometry(error,a);
    elseif mode == 2
        c = generateCovarianceVelocity(error,a);
    end
    L = 0;
    for i=1:1:N
        if abs(c(i)) > 10^(-4) && abs(x(i)) > 10^(-4)
            L = L + (-0.5*x(i)*(1/(c(i)^2))*x(i) - 0.5*log(c(i)));
        end
    end
end
function [dL] = getJacLogLikelihood(x,error,a,mode)
    % Calculates the derivation of the log likelihood given the measurements and the
    % covariance matrix assuming a zero mean gaussian distribution
    
    N = length(a);
    da = 10^(-9);
    
    L = getLogLikelihood(x,error,a,mode); 
    
    dL = zeros(N,1);
    for i=1:1:N
        a_tmp = a; a_tmp(i) = a_tmp(i) + da; 
        L_tmp = getLogLikelihood(x,error,a_tmp,mode); 
        dL(i) = (L_tmp - L)/da;
    end
end
function [c] = generateCovarianceOdometry(error,a)
    % Calculates the covariance given the actual parameters a
    M = length(error.Error(:,1));
    N = length(error.Error(1,:));
    c = zeros(N*M,1);
    for j=1:1:M
        for i=1:1:N
            if j == 1
                c(i+(j-1)*N) = abs(a(1))*abs(error.Delta.OptiTrack(1,i)) ...
                                    + abs(a(2))*abs(error.Delta.OptiTrack(2,i));
            elseif j == 2
                c(i+(j-1)*N) = (abs(a(3))*abs(error.Delta.OptiTrack(2,i)) ...
                                    + abs(a(4))*(abs(error.Delta.OptiTrack(1,i)) ...
                                    + abs(error.Delta.OptiTrack(3,i))));
            elseif j == 3
                c(i+(j-1)*N) = (abs(a(1))*abs(error.Delta.OptiTrack(3,i)) ...
                                    + abs(a(2))*abs(error.Delta.OptiTrack(2,i)));
            end
        end
    end
end
function [c] = generateCovarianceVelocity(error,a)
    % Calculates the covariance given the actual parameters a
    M = length(error.Error(:,1));
    N = length(error.Error(1,:));
    c = zeros(N*M,1);
	for j=1:1:M
        for i=1:1:N
            if j == 1
                c(i+(j-1)*N) = abs(a(1))*abs(error.Ctrl(1,i)) ...
                                    + abs(a(2))*abs(error.Ctrl(2,i));
            elseif j == 2
                c(i+(j-1)*N) = abs(a(3))*abs(error.Ctrl(1,i)) ...
                                    + abs(a(4))*abs(error.Ctrl(2,i));
            elseif j == 3
                c(i+(j-1)*N) = abs(a(5))*abs(error.Ctrl(1,i)) ...
                                    + abs(a(6))*abs(error.Ctrl(2,i));
            end
        end
    end
end

%% Optimization functions for the robot parameters
function [param] = optimizeRobotParams(odometry,optiTrack,param,mode)
    % Function to optimize the car parameter based on the measurement error
    % param = [tDelay, diameter, L];
    % mode: 1: optimize tDelay
    %       2: optimize other parameters
    
    e = getErrorRobotParam(odometry,optiTrack,param);
    diff = inf;
    alpha = 10^(-4);
    c = 0;
    while diff > 10^(-6) && c < 10
        de = getJacError(odometry,optiTrack,param,mode);
        if mode == 1
            param(1) = param(1) - alpha * de;
        elseif mode == 2
            param(2:4) = param(2:4) - alpha * de;
        end
        e_new = getErrorRobotParam(odometry,optiTrack,param);
        diff = e - e_new
        e = e_new;
        c = c + 1;
    end    
end
function e = getErrorRobotParam(odometry,optiTrack,param)
    % Get way driven by the wheels from the ticks
    x = 1;
    [odometry] = generateOdomLength(odometry,param(2:3),param(1));
    % Interpolate OptiTrackData such that times fit together
    [optiTrackInter] = interpolatingOptitrack(optiTrack,odometry);
    % Allocate odometry data to the optiTrack measurements for dN steps
    [selection] = allocateOdometryData(100, odometry, optiTrackInter, param(4));
    % Get the odometry errors
    [error] = odometryError(selection);
    % Sum up error
    e = sum(error.Error(1,:).^2) + sum(error.Error(3,:).^2) + sum(error.Error(2,:).^2); 
end
function [de] = getJacError(odometry,optiTrack,param,mode)
    % Calculates the derivation of the log likelihood given the measurements and the
    % covariance matrix assuming a zero mean gaussian distribution   
    da = 10^(-9);   
    e = getErrorRobotParam(odometry,optiTrack,param);  
    if mode == 1
        param_tmp = param; param_tmp(1) = param_tmp(1) + da; 
        e_tmp = getErrorRobotParam(odometry,optiTrack,param_tmp);
        de = (e_tmp - e)/da;
    elseif mode == 2
        de = zeros(3,1);
        for i=2:1:4
            param_tmp = param; param_tmp(i) = param_tmp(i) + da; 
            e_tmp = getErrorRobotParam(odometry,optiTrack,param_tmp);
            de(i-1) = (e_tmp - e)/da;
        end
    end
end