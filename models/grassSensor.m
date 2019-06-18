classdef GrassSensor
% Grass sensor class. Here we create the measurements based on the given
% map. A sensor gives back 1 if grass has been detected and 0 if not.
%
% Methods
%   GrassSensor(polyMap)
%       Constructor of the class
%   sensorData = measure(obj,pose)
%       gives back noise corrupted measurements data depending on the pose
%       of the robot
%
% Date:     28.11.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

    properties
        PolyMap;            % Map of the environment
        PosRight;           % Position of the right sensor relative to the body frame
        PosLeft;            % Position of the left sensor relative to the body frame
        Noise;              % Sensor noise
    end
    
    methods
        function obj = GrassSensor(polyMap)
            % Constructor
            obj.PolyMap = polyMap;
            % Load parameters
            out = get_config('Sensor');
            obj.PosRight =  out.posRight;
            obj.PosLeft = out.posLeft;
            obj.Noise = out.noise;
        end
        
        function sensorData = measure(obj,pose)
            % This function defines the sensor setting for the lawn mower
            %
            % Syntax:
            %       sensorData = grassSensor(obj,pose)
            %
            % Input:
            %   obj:            Object of the class
            %   pose:           Actual Pose of the vehicle, [x y phi]^T
            %
            % Output:
            %   sensorData:     Struct with sensor Data
            %

            % Orientation Matrix
            R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];

            % Caluclate the actual positions of the sensors
            pR = pose(1:2) + R*obj.PosRight;
            pL = pose(1:2) + R*obj.PosLeft;

            % Make the measurements
            sensorData.right = inpolygon(pR(1),pR(2),obj.PolyMap.x,obj.PolyMap.y);
            sensorData.left = inpolygon(pL(1),pL(2),obj.PolyMap.x,obj.PolyMap.y);
            
            % Corrupt measurements
            if rand() < 0.5*obj.Noise
                sensorData.right = ~sensorData.right;
            end
            if rand() < 0.5*obj.Noise
                sensorData.left = ~sensorData.left;
            end
        end
    end
end

