classdef OdometryModel
% Odometry model class
%
% Methods
%   OdometryModel()
%       Constructor of the class
%   [obj,odometryData] = odometryData(obj, p0, motionData)
%       generates the odometry data from given motionData as in (*1)
%   [p1] = odometryPose(obj, p0, noisy, incNoise)
%       generates the pose of the robot given the odometry data including
%       noise
% 
% Date:     28.11.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
%
% 1*: Probabilistic Robotics, Thrun et. al.

    properties
        L;
        A;
        DeltaR1;
        DeltaT;
        DeltaR2;
    end
    
    methods
        function obj = OdometryModel() 
            % Constructor which initializes and loads the parameters
            % required for the odometry model
            
            % Parameters
            out = get_config('mowerParameter');
            obj.L = out.L;         % half of the dimension between axes, in [m]
            
            out = get_config('odometryModelNoise');
            obj.A = out.a;  
            
            % Initialize variables
            obj.DeltaR1 = 0;
            obj.DeltaT = 0;
            obj.DeltaR2 = 0;
        end
        
        function [obj,odometryData] = odometryData(obj, p0, motionData)
            % The model as presented in "Probabilistic Robotics" is used to
            % calculate the odometry data. See page 110.
            %
            % Syntax:
            %       [odometryData] = odometryModel(p0, motionData, noisy)
            %
            % Input:
            %   p0:             Actual Position of the vehicle, [x y phi]^T
            %   motionData:     Data from the simulated encoder from the kinematic
            %                   modelas a struct
            %
            % Output:
            %   odometryData:   Odometry data as struct

            % Calculate Position Estimate from the odometry data
            ds = (motionData.lR - motionData.lL ) / 2;
            dphi = (motionData.lR + motionData.lL ) / (2*obj.L);

            % Use the odometry model from "Probabilistic Robotics" in order to add 
            % convenient noise
            dp = [cos(p0(3)), 0; sin(p0(3)), 0; 0, 1] * [ds; dphi];
            if ds >= 0
                obj.DeltaR1 = atan2(dp(2), dp(1)) - p0(3);
                obj.DeltaT = sqrt(dp(1)^2 + dp(2)^2);
                if abs(obj.DeltaR1) > abs(dp(3))     % if deltaR1 is unrealistic high, then atan2 failed
                    obj.DeltaR1 = 0.5 * dp(3);
                end
                obj.DeltaR2 = dp(3) - obj.DeltaR1;
            else
            	obj.DeltaR1 = atan2(-dp(2), -dp(1)) - p0(3);
                obj.DeltaT = -sqrt(dp(1)^2 + dp(2)^2);
                if abs(obj.DeltaR1) > abs(dp(3))     % if deltaR1 is unrealistic high, then atan2 failed
                    obj.DeltaR1 = 0.5 * dp(3);
                end
                obj.DeltaR2 = dp(3) - obj.DeltaR1;
            end
            
            odometryData.DeltaR1 = obj.DeltaR1;
            odometryData.DeltaT = obj.DeltaT;
            odometryData.DeltaR2 = obj.DeltaR2;
        end
        
        function [p1] = odometryPose(obj, p0, noisy, incNoise)
            % This function calculates the pose based on the odometry model from 
            % "Probabilistic Robotics". See page 110.
            %
            % Syntax:
            %       [odometryData] = odometryModel(p0, motionData, noisy, incNoise)
            %
            % Input:
            %   p0:             Actual Pose of the vehicle, [x y phi]^T
            %   odometryData:   Calculated odometry data
            %   noisy:          Boolean, if odometry data should be noisy
            %   incNoise:       Factor for increasing the noise
            %
            % Output:
            %   p1:             Estimated Pose

            % Add noise
            deltaR1 = obj.DeltaR1 - incNoise * sampleNormalDistribution(obj.A(1)*abs(obj.DeltaR1) + obj.A(2)*obj.DeltaT);
            deltaT = obj.DeltaT - incNoise * sampleNormalDistribution(obj.A(3)*obj.DeltaT + obj.A(4)*abs(obj.DeltaR1+obj.DeltaR2));
            deltaR2 = obj.DeltaR2 - incNoise * sampleNormalDistribution(obj.A(1)*abs(obj.DeltaR2) + obj.A(2)*obj.DeltaT);

            % Generate noisy pose estimate
            if noisy
                p1 = p0 + [cos(p0(3)+deltaR1), 0; sin(p0(3)+deltaR1), 0; 0, 1] ...
                        * [deltaT; (deltaR1+deltaR2)];
            end
        end
    end
end

