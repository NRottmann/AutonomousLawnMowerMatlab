classdef KinematicModel
% Kinematic Model class
%
% Methods
%   KinematicModel()
%       Constructor of the class
%   [p1,motionData] = kinModel(p0, u, noisy)
%       generates the motion data from given pose and input signals as in (*1)
% 
% Date:     28.11.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
%
% 1*: Probabilistic Robotics, Thrun et. al. 

    properties
        L;
        Vmax;
        Wmax;
        A;
        Dt;
    end
    
    methods
        function obj = KinematicModel()
            % Constructor
            out = get_config('mowerParameter');
            obj.L = out.L;              % half of the dimension between axes, in [m]

            out = get_config('VelLimitations');
            obj.Vmax = out.v_max;       % maximal speed in [m/s]
            obj.Wmax = out.w_max;       % maximal angular velocity in [rad/s]

            out = get_config('kinModelNoise');
            obj.A = out.a;              % Noise values

            out = get_config('system');
            obj.Dt = out.dt;            % Time intervall
            
        end
        function [p1,motionData] = kinModel(obj, p0, u, noisy)
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

            %% Check for correct dimensions
            if (size(p0) ~= [3 1])
                error('Size of input p0 is not correct!')
            end
            if (size(u) ~= [2 1])
                error('Size of input u is not correct!')
            end

            %% Kinematic Model from "Probabilistic Robotics"
            v = u(1);
            w = u(2);

            % Check boundaries
            if v > obj.Vmax; v = obj.Wmax; end
            if v < -obj.Vmax; v = -obj.Wmax; end
            if w > obj.Wmax; w = obj.Wmax; end
            if w < -obj.Wmax; w = -obj.Wmax; end

            % Add noise, TODO: think about the final term g = ...
            if noisy
                v = v + sampleNormalDistribution(obj.A(1)*abs(v) + obj.A(2)*abs(w));
                w = w + sampleNormalDistribution(obj.A(3)*abs(v) + obj.A(4)*abs(w));
                g = 0; %sampleNormalDistribution(obj.A(5)*abs(v) + obj.A(6)*abs(w));
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
                phi0 = p0(3) + g*obj.Dt;
                p1 = p0 + [v*cos(phi0)*obj.Dt; v*sin(phi0)*obj.Dt; 0];
                w = 0;
            else
                phi0 = p0(3);
                phi1 = phi0 + w*obj.Dt;
                vw = v/w;

                p1 = zeros(3,1);
                p1(1) = p0(1) + vw*(sin(phi1)-sin(phi0));
                p1(2) = p0(2) + vw*(cos(phi0)-cos(phi1));
                p1(3) = phi1 + g*obj.Dt;   
            end


            %% Wheel Movements
            vR = obj.L*w + v;
            vL = obj.L*w - v;

            % Motion left, right
            motionData.lR = vR*obj.Dt;
            motionData.lL = vL*obj.Dt;

        end
    end
end


