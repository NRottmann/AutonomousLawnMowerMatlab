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
    case 'VelLimitations'
        out.v_max = 0.3;
        out.w_max = 0.6;
    case 'sensorPositions'
        out.posRight = [0.265; -0.09];   
        out.posLeft = [0.265; 0.09];
    case 'kinModelNoise'
        out.a = [0.0012,0.001,0.0057,0.0032,0.0035,0.0046];
    case 'mowerParameter'
        out.L = 0.1845;
    case 'odometryModelNoise'
        out.a = [0.0072,0.0017,0.0010,0.0003];
    case 'particleFilter'
        out.weightFactors = [0.95,0.3,0.05];
        out.treshResample = 0.7;
    case 'system'
        out.dt = 0.05;
    case 'mapping'
        out.L_min = 0.5;
        out.e_max = 0.001;
        out.L_nh = 10;
        out.C_min = 0.3;
        out.M = 100;
        out.alpha = 20;
        out.beta = 10;
        out.gamma = 50;
        out.maxIter = 500;
        out.minDiff = 10^(-6);
end
end

