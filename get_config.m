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
        out.w_max = 0.3;
    case 'sensorPositions'
        out.posRight = [0.265; -0.09];   
        out.posLeft = [0.265; 0.09];
    case 'kinModelNoise'
        out.a = [0.1,0.1,0.1,0.1,0.1,0.1];
    case 'mowerParameter'
        out.L = 0.1845;
    case 'odometryModelNoise'
        out.a = [0.1,0.1,0.1,0.1];
    case 'particleFilter'
        out.weightFactors = [0.95,0.3,0.05];
        out.treshResample = 0.7;
    case 'system'
        out.dt = 0.05;
end
end

