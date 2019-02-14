function polyMap = genPolyMap(x,y)
% Generates a poly map structure from the given 2D points
%
% Input:
%   x:  1xN Vector with x-coordinates
%   y:  1xN Vector with y-coordinates
%
% Output:
%   polyMap:    Structure of the poly map

polyMap.x = x;
polyMap.y = y;

polyMap.XMapLimits = [min(polyMap.x) max(polyMap.x)];
polyMap.YMapLimits = [min(polyMap.y) max(polyMap.y)];

polyMap.XWorldLimits = [polyMap.XMapLimits(1)-1, polyMap.XMapLimits(2)+1];
polyMap.YWorldLimits = [polyMap.YMapLimits(1)-1, polyMap.YMapLimits(2)+1];

polyMap.Circumference = 0;
for i=1:1:length(polyMap.x)-1
    p1 = [polyMap.x(i); polyMap.y(i)];
    p2 = [polyMap.x(i+1); polyMap.y(i+1)];
    polyMap.Circumference = polyMap.Circumference + norm(p2-p1);
end

end

