function [odometryData] = odometryModel(p0, motionData)
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
    
    %% Check for correct dimensions
    if (size(p0) ~= [3 1])
        error('Size of input p0 is not correct!')
    end
    
    %% Parameters
    out = get_config('mowerParameter');
    L = out.L;         % half of the dimension between axes, in [m]
    
    %% Calculate Position Estimate from the odometry data
    ds = (motionData.lR - motionData.lL ) / 2;
    dphi = (motionData.lR + motionData.lL ) / (2*L);
    
    % TODO: is p0 really required or can we simply use p = [0,0,0]?
    p1 = p0 + [cos(p0(3)), 0; sin(p0(3)), 0; 0, 1] * [ds; dphi];
      
    %% Use the odometry model from "Probabilistic Robotics" in order to add 
    % convenient noise
    odometryData.deltaR1 = atan2(p1(2) - p0(2), p1(1) - p0(1)) - p0(3);
    odometryData.deltaT = sqrt((p0(1)-p1(1))^2 + (p0(2)-p1(2))^2);
    if abs(odometryData.deltaR1) > 0.1     % if deltaR1 is unrealistic high, then atan2 failed
        odometryData.deltaR1 = 0.5 * (p1(3) - p0(3));
    end
    odometryData.deltaR2 = p1(3) - p0(3) - odometryData.deltaR1;
end

