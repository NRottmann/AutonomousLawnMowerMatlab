classdef RandomController
    % This is a cntrol class for a random controller which simply drives
    % the vehicle forward until a sensor detects no grass, the it decides
    % randomly how long the vehicle is turned
    %
    % Date:     04.10.2018
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
            obj.MeanSL = 0.9;
            obj.Mode = 0;
            obj.AngleDiff = 0;
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
                        obj.AngleDiff = obj.AngleDiff - min([obj.AngleDiff,(odometryData.deltaR1 + odometryData.deltaR2)]);
                    else
                        obj.AngleDiff = obj.AngleDiff + min([-1*obj.AngleDiff, -1*(odometryData.deltaR1 + odometryData.deltaR2)]);
                    end
                end
            end
        end
    end
end

