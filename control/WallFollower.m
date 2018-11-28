classdef WallFollower
    % This is a control class for a wall follower which simply drives
    % the vehicle along the boundary line, starting by first searching for 
    % the boundary. The wall follower drives the vehicle counter clockwise
    % along the boundary line using only the sensor measurements of the
    % right sensor
    %
    % Methods
    %   WallFollower()
    %       Constructor of the class
    %   [obj,u] = wallFollowing(obj,sensorData)
    %       generates the input data based on the sensor measurement
    %
    % Date:     28.11.2018
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    
    properties
        Mode;                   % Mode of the controller, either 0,1 (searching, wall following)
        Mean;                   % Mean value for wall following
        Count;                  % Angle at which the robot has to turn
        Vel;                    % Current procentual velocity
        Vmax;                   % Maximum linear and angular velocity
        Wmax;
        A_mu;                   % Parameters for exponential smoothing
        A_v;
        M;
    end
    
    methods    
        function obj = WallFollower()
            % Constructor
            
            % Get parameters
            out = get_config('VelLimitations');
            obj.Vmax = out.v_max;
            obj.Wmax = out.w_max;
            
            out = get_config('WallFollower');
            obj.A_mu = out.a_mu;
            obj.A_v = out.a_v;
            obj.M = out.M;
            
            % Initialize variables
            obj.Mean = 1;       % We assume we start within the polygon map
            obj.Count = 0;
            obj.Mode = 0;
            obj.Vel = 0;
        end
        
        function [obj,u] = wallFollowing(obj,sensorData)
            % Syntax:
            %       [u] = wallFollowing(sensorData)
            %
            % Input:
            %   sensorData:     Data of the grass sensor
            %
            % Output:
            %   u:              Control inputs for kinematic model
         
            % count up
            obj.Count = obj.Count + 1;

            % Procede measurements
            if sensorData.right
                SRmeas = 1;
            else
                SRmeas = 0;
            end

            % Initialize control signals
            u = zeros(2,1);

            % Start finding the border line
            if obj.Mode == 0
                obj.Mean = obj.A_mu * obj.Mean + (1-obj.A_mu) * SRmeas;
                if obj.Mean > 0.3
                   u(1) = obj.Vmax;
                else
                   obj.Mode = 1;
                end
            end

            % Do the wall following
            if (obj.Mode == 1)
                obj.Mean = obj.A_mu * obj.Mean + (1-obj.A_mu) * SRmeas;
                d = (0.5 - obj.Mean) * 2.0;
                var = cos(2 * pi * (obj.Count/obj.M));
                u(2) = (d + var) * 0.5 * obj.Wmax;
                obj.Vel = obj.A_v * obj.Vel + (1-obj.A_v) * (1 - abs(d));
                u(1) = obj.Vel * obj.Vmax;
            end
        end
    end
end

