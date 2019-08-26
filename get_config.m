function out = get_config(caseString)
% This function gives back the configuration required for different
% setting, e.g. PD controller, the odometry, etc.

out = [];
switch caseString
    case 'PDControl'
        out.Kp = 10;
        out.Kv = 1;
        out.Kr = 1;
        out.Kw = 1;
    case 'WallFollower'
        out.a_mu = 0.7;
        out.a_v = 0.7;
        out.M = 100;
    case 'VelLimitations'
        out.v_max = 0.3;
        out.w_max = 0.6;
    case 'Sensor'
        out.posRight = [0.265; 0.0];
        % out.posRight = [0.265; -0.09];    % The right sensor is used for the wall follower      
        out.posLeft = [0.265; 0.09];
        out.noise = 0.1;        % Noise of the sensor, 0 means no noise, 1 means totally random
    case 'kinModelNoise'
        % out.a = [0.0012,0.001,0.0057,0.0032,0.0035,0.0046];     
        out.a = [sqrt(0.0012),sqrt(0.001),sqrt(0.0057),sqrt(0.0032),sqrt(0.0035),sqrt(0.0046)];     % Standard deviation parameters
        % out.a = [0.1,0.1,0.1,0.1,0.1,0.1];
%         out.a = [0.0,0.0,0.0,0.0,0.0,0.0];
%         out.a = [0.021,0.01,0.01,0.01,0.01,0.01];
    case 'mowerParameter'
        out.L = 0.1826;
        out.dR = 0.2147;
        out.dL = 0.2144;
    case 'odometryModelNoise'
       % out.a = [0.0254, 0.0111, 0.0107, 0.0097];
        out.a = [0.0849, 0.0412, 0.0316, 0.0173];   % Standard deviation parameters
        % out.a = [0.4, 0.4, 0.4, 0.4];
%         out.a = [0.0, 0.0, 0.0, 0.0];
%         out.a = [0.01,0.01,0.01,0.01,0.01,0.01];
    case 'system'
        out.dt = 0.05;
    case 'mapping'
        out.l_min = 0.05;
        out.e_max = 0.001; %0.001
        out.l_nh = 44.6644; %30;
        out.c_min = 0.50665; %1.0;
        out.phi_cycle = 1.5;
        out.M = 100;
        out.gamma1 = 1;
        out.gamma2 = 1;
        out.bayRate = 1000;
	case 'globalLocalization'
        out.l_min = 0.1;
        out.e_max = 0.001;
        out.u_min = 0.5;
        out.l_nh = 30;
        out.c_min = 1.0;
        out.c_diff = 0;
    case 'particleFilter'
        out.n_P = 250;                      % Number particles
        out.poseVariance = [0.5;0.5;0.5];   % Variance for distributing the particles around a initial pose estimate
        out.n_M = 1;                        % Measure n_M times before updating weights of the particle filter
        out.increaseNoise = 1;            	% Factor which increases the noise of the odometry model for the particles
        out.n_S = 2;                      	% Number sensors used, (1 or 2)
        out.thresholdResampling = 0.9;   	% Resampling treshold  
    case 'coverageMap'
        out.resolution = 5;                % Resolution in cells per meter
        out.threshhold = 0.9;
        out.wallFollow = 0.4;
    case 'planning'
        out.a = 10;          % Passive decay rate
        out.b = 1;            % Upper bound
        out.d = 1;          % Lower bound
        out.e = 100;              % Maximum Gain, for gradient
        out.c = 3;          % Control gain
        out.threshhold = 0.9;
        out.dt = 0.01;
        out.g = 2*pi+1;
end
end

