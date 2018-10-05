classdef GrassSensor
% Grass sensor class
% Methods
% ...
% Date:     04.10.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

    properties
        PolyMap;
        PosRight;
        PosLeft;
    end
    
    methods
        function obj = GrassSensor(polyMap)
            obj.PolyMap = polyMap;
            % Position of the sensors in the coordinates of the lawn mower
            out = get_config('sensorPositions');
            obj.PosRight =  out.posRight;
            obj.PosLeft = out.posLeft; 
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
            %   sensorData: Struct with sensor Data
            %

            % Orientation Matrix
            R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];

            % Caluclate the actual positions of the sensors
            pR = pose(1:2) + R*obj.PosRight;
            pL = pose(1:2) + R*obj.PosLeft;

            % Make the measurements
            sensorData.right = inpolygon(pR(1),pR(2),obj.PolyMap.x,obj.PolyMap.y);
            sensorData.left = inpolygon(pL(1),pL(2),obj.PolyMap.x,obj.PolyMap.y);
        end
    end
end

