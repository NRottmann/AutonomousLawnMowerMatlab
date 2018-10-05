classdef WallFollower
    % This is a control class for a wall follower which simply drives
    % the vehicle along the boundary line, starting by first searching the
    % boundary
    %
    % Date:     04.10.2018
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    
    properties
        Mode;                   % Mode of the controller, either 0,1 (searching, wall following)
        Mean;                   % Mean value for wall following
        Count;                  % Angle at which the robot has to turn
        Vel;                    % Current procentual velocity
        Vmax;                   % Maximum linear and angular velocity
        Wmax;
    end
    
    methods
        
        function obj = WallFollower()
            % Constructor
            
            % Get parameters
            out = get_config('VelLimitations');
            obj.Vmax = out.v_max;
            obj.Wmax = out.w_max;
            
            % Initialize variables
            obj.Mean = 0;
            obj.Count = 0;
            obj.Mode = 0;
            obj.Vel = 0;
        end
        
        function [obj,u] = wallFollowing(obj,sensorData)
            % Syntax:
            %       [u] = randomControl(sRight,sLeft)
            %
            % Input:
            %   sensorData:     Data of the grass sensor
            %   odometryData:   Data of the wheels
            %
            % Output:
            %   u:              Control inputs for kinematic model
            
            % Parameters
            M = 30;

            % count up
            obj.Count = obj.Count + 1;

            % Procede measurements
            if sensorData.right
                SRmeas = 1;
            else
                SRmeas = 0;
            end
            if sensorData.left
                SLmeas = 1;
            else
                SLmeas = 0;
            end

            % Initialize control signals
            u = zeros(2,1);

            % Start finding the border line
            if ((SLmeas == 1 || SRmeas == 1) && (obj.Mode == 0))
                u(1) = obj.Vmax;
            elseif (SLmeas == 0 && SRmeas == 0)
                obj.Mode = 1;
            end

            % Do the wall following
            if (obj.Mode == 1)
                obj.Mean = 0.9 * obj.Mean + 0.1 * SRmeas;
                d = (0.5 - obj.Mean) * 2.0;
                var = cos(2 * pi * (obj.Count/M));
                u(2) = (d + var) * 0.5 * obj.Wmax;
                obj.Vel = 0.9 * obj.Vel + 0.1 * (1 - abs(d));
                u(1) = obj.Vel * obj.Vmax;
            end
        end
    end
end

