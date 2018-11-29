function out = get_config(caseString)
% This function gives back the configuration required for different
% setting, e.g. PD controller, the odometry, etc.

out = [];
switch caseString
    case 'PDControl'
        out.Kp = 1;
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
        % out.posRight = [0.265; -0.09];
        out.posRight = [0.3; 0];
        out.posLeft = [0.265; 0.09];
        out.noise = 0.1;        % Noise of the sensor, 0 means no noise, 1 means totally random
    case 'kinModelNoise'
        % out.a = [0.0012,0.001,0.0057,0.0032,0.0035,0.0046];     % Standard deviation parameters
        out.a = [sqrt(0.0012),sqrt(0.001),sqrt(0.0057),sqrt(0.0032),sqrt(0.0035),sqrt(0.0046)];     % Standard deviation parameters
        % out.a = [0.1,0.1,0.1,0.1,0.1,0.1];
    case 'mowerParameter'
        out.L = 0.1845;
    case 'odometryModelNoise'
        % out.a = [0.0072,0.0017,0.0010,0.0003];
        out.a = [sqrt(0.0072),sqrt(0.0017),sqrt(0.0010),sqrt(0.0003)];
        % out.a = [0.1,0.1,0.1,0.1];
    case 'particleFilter'
        out.weightFactors = [1.0,0.1,0.01];
        out.treshResample = 0.9;
    case 'system'
        out.dt = 0.05;
    case 'mapping'
        out.l_min = 0.1;
        out.e_max = 0.00001;
        out.l_nh = 15;
        out.c_min = 0.15;
        out.M = 100;
        out.alpha = 20;
        out.beta = 10;
        out.gamma = 50;
        out.maxIter = 500;
        out.minDiff = 10^(-6);
end
end

