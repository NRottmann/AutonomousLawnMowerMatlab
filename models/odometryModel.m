classdef OdometryModel
% Odometry model class
% Methods
% ...
% Date:     04.10.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

    properties
        L;
        A;
        DeltaR1;
        DeltaT;
        DeltaR2;
    end
    
    methods
        function obj = OdometryModel()      
            %% Parameters
            out = get_config('mowerParameter');
            obj.L = out.L;         % half of the dimension between axes, in [m]
            
            out = get_config('odometryModelNoise');
            obj.A = out.a;  
            
            %% Initialize variables
            obj.DeltaR1 = 0;
            obj.DeltaT = 0;
            obj.DeltaR2 = 0;
        end
        
        function obj = odometryData(obj, p0, motionData)
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
            %
            % Date:             01.10.2018
            % Author:           Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

            %% Calculate Position Estimate from the odometry data
            ds = (motionData.lR - motionData.lL ) / 2;
            dphi = (motionData.lR + motionData.lL ) / (2*obj.L);
 
            p1 = p0 + [cos(p0(3)), 0; sin(p0(3)), 0; 0, 1] * [ds; dphi];

            %% Use the odometry model from "Probabilistic Robotics" in order to add 
            % convenient noise
            obj.DeltaR1 = atan2(p1(2) - p0(2), p1(1) - p0(1)) - p0(3);
            obj.DeltaT = sqrt((p0(1)-p1(1))^2 + (p0(2)-p1(2))^2);
            if abs(obj.DeltaR1) > 0.1     % if deltaR1 is unrealistic high, then atan2 failed
                obj.DeltaR1 = 0.5 * (p1(3) - p0(3));
            end
            obj.DeltaR2 = p1(3) - p0(3) - obj.DeltaR1;
        end
        
        function [p1] = odometryPose(obj, p0, noisy)
            % This function calculates the pose based on the odometry model from 
            % "Probabilistic Robotics". See page 110.
            %
            % Syntax:
            %       [odometryData] = odometryModel(p0, motionData, noisy)
            %
            % Input:
            %   p0:             Actual Pose of the vehicle, [x y phi]^T
            %   odometryData:   Calculated odometry data
            %   noisy:          Boolean, if odometry data should be noisy
            %
            % Output:
            %   p1:             Estimated Pose
            %
            % Date:             01.10.2018
            % Author:           Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

            % Add noise
            deltaR1 = obj.DeltaR1 - sampleNormalDistribution(obj.A(1)*abs(obj.DeltaR1) + obj.A(2)*obj.DeltaT);
            deltaT = obj.DeltaT - sampleNormalDistribution(obj.A(3)*obj.DeltaT + obj.A(4)*abs(obj.DeltaR1+obj.DeltaR2));
            deltaR2 = obj.DeltaR2 - sampleNormalDistribution(obj.A(1)*abs(obj.DeltaR2) + obj.A(2)*obj.DeltaT);

            % Generate noisy pose estimate
            if noisy
                p1 = p0 + [cos(p0(3)+deltaR1), 0; sin(p0(3)+deltaR1), 0; 0, 1] ...
                        * [deltaT; (deltaR1+deltaR2)];
            end
        end
    end
end

