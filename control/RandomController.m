classdef RandomController
    % This is a cntrol class for a random controller which simply drives
    % the vehicle forward until a sensor detects no grass, the it decides
    % randomly how long the vehicle is turned.
    %
    % Methods
    %   RandomController()
    %       Constructor of the class
    %   [obj,u] = randomControl(obj,sensorData,odometryData)
    %       generates the input data based on the sensor measurements and
    %       odometry data based only on the right sensor
    %   [obj,u] = randomControlDouble(obj,sensorData,odometryData)
    %       generates the input data based on the sensor measurements and
    %       odometry data based on two sensors (right and left)
    %
    % Date:     28.11.2018
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    
    properties
        Mode;                   % Mode of the controller, either 0,1 (forward, rotate)
        MeanSR;                 % Mean values of measurements (right and left sensor)
        MeanSL;
        AngleDiff;              % Angle at which the robot has to turn
        Vmax;                   % Maximum linear and angular velocity
        Wmax;
    end
    
    methods
        
        function obj = RandomController()
            % Constructor
            
            % Get parameters
            out = get_config('VelLimitations');
            obj.Vmax = out.v_max;
            obj.Wmax = out.w_max;
            
            % Initialize variables
            obj.MeanSR = 0.9;
            obj.Mode = 1;
            obj.AngleDiff = pi/2;
        end
        
        function [obj,u] = randomControl(obj,sensorData,odometryData)
            % Syntax:
            %       [u] = randomControl(sRight,sLeft)
            %
            % Input:
            %   sensorData:     Data of the grass sensor
            %   odometryData:   Data of the wheels
            %
            % Output:
            %   u:              Control inputs for kinematic model
            
            % TODO: Improve algorithm, such that it works also for large dt
            
            % Procede measurements
            if sensorData.right
                SRmeas = 1;
            else
                SRmeas = 0;
            end
            obj.MeanSR = obj.MeanSR*0.9 + SRmeas*0.1;
            
            % Define output
            u = zeros(2,1);
            
           	% Move forward
            if (obj.Mode == 0) 
                % One of the sensor detect nothing
                if (obj.MeanSR < 0.5)
                    obj.Mode = 1;           % Rotate
                    obj.AngleDiff = rand() * pi/2;
                else
                    % No turning
                    u(1) = obj.Vmax;
                end
            end

            % Rotate 
            if (obj.Mode == 1) 
                if (abs(obj.AngleDiff) < 0.1) 
                    obj.Mode = 0;          	% Straight
                    obj.MeanSR = 0.9;
                else
                    u(2) = obj.Wmax;
                    obj.AngleDiff = obj.AngleDiff - min([obj.AngleDiff,(odometryData.DeltaR1 + odometryData.DeltaR2)]);
                end
            end
        end
        
        function [obj,u] = randomControlDouble(obj,sensorData,odometryData)
            % Syntax:
            %       [u] = randomControl(sRight,sLeft)
            %
            % Input:
            %   sensorData:     Data of the grass sensor
            %   odometryData:   Data of the wheels
            %
            % Output:
            %   u:              Control inputs for kinematic model
            
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
            obj.MeanSR = obj.MeanSR*0.9 + SRmeas*0.1;
            obj.MeanSL = obj.MeanSL*0.9 + SLmeas*0.1;
            
            % Define output
            u = zeros(2,1);
            
           	% Move forward
            if (obj.Mode == 0) 
                % One of the sensor detect nothing
                if (obj.MeanSR < 0.8 || obj.MeanSL < 0.8)
                    if ((obj.MeanSR - obj.MeanSL) > 0)
                        angleDiff = (2*pi/360) * (60 + rand()*60);
                    else
                        angleDiff = (2*pi/360) * (-120 + rand()*60);
                    end
                    obj.AngleDiff = angleDiff;
                    obj.Mode = 1;           % Rotate
                else
                    % No turning
                    u(1) = obj.Vmax;
                end
            end

            % Rotate 
            if (obj.Mode == 1) 
                if (abs(obj.AngleDiff) < 0.1) 
                    obj.Mode = 0;          	% Straight
                    obj.MeanSR = 0.9;
                    obj.MeanSL = 0.9;
                else
                    if obj.AngleDiff > 0
                        u(2) = obj.Wmax;
                    else
                        u(2) = -obj.Wmax;
                    end
                    if (obj.AngleDiff > 0)
                        obj.AngleDiff = obj.AngleDiff - min([obj.AngleDiff,(odometryData.DeltaR1 + odometryData.DeltaR2)]);
                    else
                        obj.AngleDiff = obj.AngleDiff + min([-1*obj.AngleDiff, -1*(odometryData.DeltaR1 + odometryData.DeltaR2)]);
                    end
                end
            end
        end
    end
end

