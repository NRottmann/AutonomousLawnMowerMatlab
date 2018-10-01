function [sensorData] = grassSensor(p,polyMap)
% This function defines the sensor setting for the lawn mower
%
% Syntax:
%       [sensorData] = grassSensor(p,mapPoly)
%
% Input:
%   p:          Actual Pose of the vehicle, [x y phi]^T
%   polyMap:    The map as Polygon
%
% Output:
%   sensorData: Struct with sensor Data
%
% Date:     01.10.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

% Position of the sensors in the coordinates of the lawn mower
out = get_config('sensorPositions');
posRight =  out.posRight;
posLeft = out.posLeft;

% Orientation Matrix
R = [cos(p(3)) -sin(p(3)); sin(p(3)) cos(p(3))];

% Caluclate the actual positions of the sensors
pR = p(1:2) + R*posRight;
pL = p(1:2) + R*posLeft;

% Define variables
sensorData.left = 0;
sensorData.right = 0;

% Make the measurements
sensorData.right = inpolygon(pR(1),pR(2),polyMap.x,polyMap.y);
sensorData.left = inpolygon(pL(1),pL(2),polyMap.x,polyMap.y);

end

