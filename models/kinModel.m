function [p1,motionData] = kinModel(p0, u, noisy)
    % This the kinematic model for the lawn mower. As input signals, the
    % lawn mower receives u = [v w], thus the velocity in the direction of
    % the lawn mower and the angular velocity. The lawn mower kinematics
    % are based of a differential drive system
    %
    % Equation
    %    dx/dt = cos(phi)*v
    %    dy/dt = sin(phi)*v
    %    dphi/dt = w
    %
    % Syntax:
    %       [p1,lR,lL] = kinModel(p0, u, dt)
    %
    % Input:
    %   p0:     Actual Position of the vehicle, [x y phi]^T
    %   u:      Input signal, [v w]^T
    %   noisy:      Boolean, if true, we add noise to odometry data
    %               (usefull for Particle Filter)
    %
    % Output:
    %   p1:             Pose after movement
    %   motionData:     Movement of the right and left wheel as struct
    %
    % Date:     01.10.2018
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    
    %% Check for correct dimensions
    if (size(p0) ~= [3 1])
        error('Size of input p0 is not correct!')
    end
    if (size(u) ~= [2 1])
        error('Size of input u is not correct!')
    end

    %% Parameters 
    out = get_config('mowerParameter');
    L = out.L;         % half of the dimension between axes, in [m]
    
    out = get_config('VelLimitations');
    v_max = out.v_max;      % maximal speed in [m/s]
    w_max = out.w_max;      % maximal angular velocity in [rad/s]
    
    out = get_config('kinModelNoise');
    a = out.a;              % Noise values
    
    out = get_config('system');
    dt = out.dt;   
    
    
    %% Kinematic Model from "Probabilistic Robotics"
    v = u(1);
    w = u(2);
   
    % Check boundaries
    if v > v_max; v = v_max; end
    if v < -v_max; v = -v_max; end
    if w > w_max; w = w_max; end
    if w < -w_max; w = -w_max; end
    
    % Add noise, TODO: think about the final term g = ...
    if noisy
        v = v + sampleNormalDistribution(a(1)*abs(v) + a(2)*abs(w));
        w = w + sampleNormalDistribution(a(3)*abs(v) + a(4)*abs(w));
        g = sampleNormalDistribution(a(5)*abs(v) + a(6)*abs(w));
    else
        g = 0;
    end
    
    % We assume there is no movement in -v direction possible, we have to
    % make this assumptions due to the restrictions from our odometry model
    if v < 0
        v = 0;
    end
    
    % Handle singularity
    if (abs(w) < 10^(-6))
        phi0 = p0(3) + g*dt;
        p1 = p0 + [v*cos(phi0)*dt; v*sin(phi0)*dt; 0];
        w = 0;
    else
        phi0 = p0(3);
        phi1 = phi0 + w*dt;
        vw = v/w;
        
        p1 = zeros(3,1);
        p1(1) = p0(1) + vw*(sin(phi1)-sin(phi0));
        p1(2) = p0(2) + vw*(cos(phi0)-cos(phi1));
        p1(3) = phi1 + g*dt;   
    end
    
    
    %% Wheel Movements
    vR = L*w + v;
    vL = L*w - v;
    
    % Motion left, right
    motionData.lR = vR*dt;
    motionData.lL = vL*dt;

end