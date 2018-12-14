% This script analyzes the odometry received from the robot and calculates
% based on the true poses (optiTrack measurements) the robot specific
% parameters
%
% Authot:   Nils Rottmann
% E-Mail:   Nils.Rottmann@rob.uni-luebeck.de
%
% Support:  Rico Klinckenberg

close all
clear all
clc

%% Parameter
% TODO: Can we also improve this parameter?
tDelay = -6.850;
diameter = [0.213, 0.2128];
L = 0.1875; % besser: 0.182
dN = 1;     % TODO: Delete this dN and set everywhere to 1!
a = [0.01; 0.01; 0.01; 0.01];       % Start parameters for optimization

plotFlag = true;

%% Load the data
numDataSet = 1;
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
    [odometry] = processBagData(bagData);
    disp('Get odometry ticks from the bag data!')
    save(['realData/parameterEvaluation',num2str(numDataSet),'.mat'],'optiTrack','odometry')
    disp('Saved Data to .mat file!')
else
    error(['File parameterEvaluation',num2str(numDataSet),'.bag or parameterEvaluation',num2str(numDataSet),'.csv does not exist!'])
end

%% Find convenient car parameters
param = [diameter, L];
[param] = optimizeCarParams(odometry,optiTrack,tDelay,param');
    
%% Adjust odometry data and optiTrack data such that they are fit together
% Get way driven by the wheels from the ticks
[odometry] = generateOdomLength(odometry,diameter,tDelay);
% Interpolate OptiTrackData such that times fit together
[optiTrackInter] = interpolatingOptitrack(optiTrack,odometry);
% Calculate odometry pose estimates
[odometry] = getOdometryPose(optiTrackInter, odometry, L);

%% Plot data if required
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

%% Allocate odometry data to the optiTrack measurements for dN steps
[selection] = allocateOdometryData(dN,odometry, optiTrackInter, L);

%% Get the odometry errors
[error] = odometryError(selection);

%% Estimate the model parameters
[a] = estimateParameter(error,a);
    
%% Functions
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

function [odometry] = processBagData(bag)

    % Parameters, TODO: Put this into the config
    maxCount = 65535;       % Max number of counts, 2 Bytes
    nHall = 703;            % Ticks per wheel rotation
    treshold = 10*nHall;    % Treshold to determine overflow of register

    % Load Odometry data
    start = bag.StartTime;
    stop = bag.EndTime;
    
    % Select the odometry data
    bagselectOdo = select(bag, 'Time', [start stop], 'Topic', '/odometryData');
    
    % Get the raw odometry data (the actual ticks number at time t)
    odo = timeseries(bagselectOdo, 'RawR', 'RawL');
   
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

function [optiTrackInter] = interpolatingOptitrack(optiTrack,odometry)
    % Interpolating the optiTrack Data
    % Getting interpolated OptiTrack positions based on Odometry Positions
    optiTrackInter(:,1) = interp1(optiTrack.Time,optiTrack.Pose(:,1),odometry.Time);
    optiTrackInter(:,2) = interp1(optiTrack.Time,optiTrack.Pose(:,2),odometry.Time);
    optiTrackInter(:,3) = interp1(optiTrack.Time,optiTrack.Pose(:,3),odometry.Time);

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

function [selection] = allocateOdometryData(dN,odometry, optiTrackInter, L)
    % Allocates the odometry data to the optiTrack data over dN steps
    
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


function [a] = estimateParameter(error,a)
    % This function uses the log likelihood method to estimate a convenient
    % set of parameters for the odometry model
    
    % Generate the error vector
    N = length(error.Error(1,:));
    M = length(error.Error(:,1));
    x = zeros(M*N,1);
    for i=1:1:N
        for j=1:1:M
            x(j+(i-1)*M) = error.Error(j,i);
        end
    end
    
    % Get the covariance matrix and calculate the current log likelihood
    L = getLogLikelihood(x,error,a);
    diff = inf;
    alpha = 10^(-8);
    c = 0;
    
    while diff > 10^(-9) && c < 10000
        dL = getJacLogLikelihood(x,error,a);
        a = a + alpha * dL;
        L_new = getLogLikelihood(x,error,a);
        diff = L_new - L;
        L = L_new;
        c = c + 1;
    end    
end

function [L] = getLogLikelihood(x,error,a)
    % Calculates the log likelihood given the measurements and the
    % covariance matrix assuming a zero mean gaussian distribution
    N = length(x);
    c = generateCovariance(error,a);
    L = 0;
    for i=1:1:N
        L = L + (-0.5*x(i)*(1/(c(i)^2))*x(i) - 0.5*log(c(i)));
    end
end

function [dL] = getJacLogLikelihood(x,error,a)
    % Calculates the derivation of the log likelihood given the measurements and the
    % covariance matrix assuming a zero mean gaussian distribution
    
    N = length(a);
    da = 10^(-9);
    
    L = getLogLikelihood(x,error,a); 
    
    dL = zeros(N,1);
    for i=1:1:N
        a_tmp = a; a_tmp(i) = a_tmp(i) + da; 
        L_tmp = getLogLikelihood(x,error,a_tmp); 
        dL(i) = (L_tmp - L)/da;
    end
end

function [c] = generateCovariance(error,a)
    % Calculates the covariance given the actual parameters a
    N = length(error.Error(1,:));
    M = length(error.Error(:,1));
    c = zeros(M*N,1);
    for i=1:1:N
        for j=1:1:M
            if j == 1
                c(j+(i-1)*M) = abs(a(1))*abs(error.Delta.OptiTrack(1,i)) ...
                                    + abs(a(2))*abs(error.Delta.OptiTrack(2,i));
            elseif j == 2
                c(j+(i-1)*M) = (abs(a(3))*abs(error.Delta.OptiTrack(2,i)) ...
                                    + abs(a(4))*(abs(error.Delta.OptiTrack(1,i)) ...
                                    + abs(error.Delta.OptiTrack(3,i))));
            elseif j == 3
                c(j+(i-1)*M) = (abs(a(1))*abs(error.Delta.OptiTrack(3,i)) ...
                                    + abs(a(2))*abs(error.Delta.OptiTrack(2,i)));
            end
        end
    end
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

function [param] = optimizeCarParams(odometry,optiTrack,tDelay,param)
    % Function to optimize the car parameter based on the measurement error
    % param = [tDelay, diameter, L];
    
    e = getErrorParam(odometry,optiTrack,tDelay,param);
    diff = inf;
    alpha = 10^(-4);
    c = 0;
    while diff > 10^(-6) && c < 1000
        de = getJacError(odometry,optiTrack,tDelay,param);
        param = param - alpha * de;
        e_new = getErrorParam(odometry,optiTrack,tDelay,param);
        diff = e - e_new;
        e = e_new;
        c = c + 1
    end    
    
    
end

function e = getErrorParam(odometry,optiTrack,tDelay,param)
    % Get way driven by the wheels from the ticks
    [odometry] = generateOdomLength(odometry,param(1:2),tDelay);
    % Interpolate OptiTrackData such that times fit together
    [optiTrackInter] = interpolatingOptitrack(optiTrack,odometry);
    % Allocate odometry data to the optiTrack measurements for dN steps
    [selection] = allocateOdometryData(100, odometry, optiTrackInter, param(3));
    % Get the odometry errors
    [error] = odometryError(selection);
    
    e = sum(error.Error(1,:).^2) + sum(error.Error(3,:).^2) + sum(error.Error(2,:).^2); 
end

function [de] = getJacError(odometry,optiTrack,tDelay,param)
    % Calculates the derivation of the log likelihood given the measurements and the
    % covariance matrix assuming a zero mean gaussian distribution
    
    N = length(param);
    da = 10^(-9);
    
    e = getErrorParam(odometry,optiTrack,tDelay,param);
    
    de = zeros(N,1);
    for i=1:1:N
        param_tmp = param; param_tmp(i) = param_tmp(i) + da; 
        e_tmp = getErrorParam(odometry,optiTrack,tDelay,param_tmp);
        de(i) = (e_tmp - e)/da;
    end
end