function [p1] = odometryPose(p0, odometryData, noisy)
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
    
    % Get parameters
    out = get_config('odometryModelNoise');
    a = out.a;  
    
    % Add noise
    deltaR1 = odometryData.deltaR1 - sampleNormalDistribution(a(1)*abs(odometryData.deltaR1) + a(2)*odometryData.deltaT);
    deltaT = odometryData.deltaT - sampleNormalDistribution(a(3)*odometryData.deltaT + a(4)*abs(odometryData.deltaR1+odometryData.deltaR2));
    deltaR2 = odometryData.deltaR2 - sampleNormalDistribution(a(1)*abs(odometryData.deltaR2) + a(2)*odometryData.deltaT);
    
    % Generate noisy pose estimate
    if noisy
        p1 = p0 + [cos(p0(3)+deltaR1), 0; sin(p0(3)+deltaR1), 0; 0, 1] ...
                * [deltaT; (deltaR1+deltaR2)];
    end
end