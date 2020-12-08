function [gridMap] = genGridMap(polyMap,res)

% Generates an occupancy grid map based on a given polygon map
gridMap = binaryOccupancyMap(polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1), ...
                                polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1),res);
                            
% Specify grid world location
gridMap.GridLocationInWorld(1) = polyMap.XWorldLimits(1);
gridMap.GridLocationInWorld(2) = polyMap.YWorldLimits(1);


for i=1:1:gridMap.GridSize(1)
    for j=1:1:gridMap.GridSize(2)
        xy = grid2world(gridMap, [i j]);
        isOccupied = inpolygon(xy(1),xy(2),polyMap.x,polyMap.y);
        setOccupancy(gridMap,xy,isOccupied);
    end
end

end

