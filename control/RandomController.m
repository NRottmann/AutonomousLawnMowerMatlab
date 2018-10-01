classdef RandomController
    % This is a cntrol class for a random controller which simply drives
    % the vehicle forward until a sensor detects no grass, the it decides
    % randomly how long the vehicle is turned
    %
    % Date:     01.010.2018
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    
    properties
        ts = 0;                 % Random turning time after no grass detection
        detectionSide = 0;      % 0: no detection, 1: first right no grass, 2: first left no grass
        counter = 0;            % counter for turning
    end
    
    methods
        function [obj,u] = randomControl(obj,sensorData)
            % Syntax:
            %       [u] = randomControl(sRight,sLeft)
            %
            % Input:
            %   sensorData:     Data of the grass sensor
            %
            % Output:
            %   u:              Control inputs for kinematic model
            
            % Get parameters
            out = get_config('VelLimitations');
            v_max = out.v_max;
            w_max = out.w_max;
            
            out = get_config('system');
            dt = out.dt;
            
            % Do the control decision
            if (obj.detectionSide == 1) && (obj.counter < obj.ts)   % turn left, since right detected
                u = [0; w_max];
                obj.counter = obj.counter + dt;
            elseif (obj.detectionSide == 2) && (obj.counter < obj.ts)   % turn right, since left detected
                u = [0; -w_max];
                obj.counter = obj.counter + dt;
            elseif (obj.counter >= obj.ts) && (obj.ts > 0)      % Rotation done, go forward
                obj.detectionSide = 0;
                obj.counter = 0;
                u = [v_max; 0];
            elseif not(sensorData.right) && not(sensorData.left)        % both sensors detect no grass, thus start turning
                % turn left
                obj.detectionSide = 1;
                obj.ts = (pi/2)/w_max + rand()*pi;
                u = [0; w_max];
                obj.counter = obj.counter + dt;
            elseif not(sensorData.left) && sensorData.right             % Left detects no grass, thus start turning right
                obj.detectionSide = 2;
                obj.ts = (pi/2)/w_max + rand()*pi/2;
                u = [0; -w_max];
                obj.counter = obj.counter + dt;
            elseif not(sensorData.right) && sensorData.left             % right detects no grass, thus start turning left
                obj.detectionSide = 1;
                obj.ts = (pi/2)/w_max + rand()*pi/2;
                u = [0; w_max];
                obj.counter = obj.counter + dt;
            else    % Grass detection, thus go forward
                u = [v_max; 0];
            end

        end
    end
end

