function [Abs] = mapAbsolute(map, Resolution)
N = round((map.XWorldLimits(2) - map.XWorldLimits(1)) * Resolution);
M = round((map.YWorldLimits(2) - map.YWorldLimits(1)) * Resolution);
Abs = zeros(N,M);

if (isa(map,'binaryOccupancyMap'))
    for i=1:1:map.GridSize(1)
        for j=1:1:map.GridSize(2)
            Abs(i,j) = getOccupancy(map,[i,j],"grid");
        end
    end
else
    stepSize = 1/Resolution;
    x_s = map.XWorldLimits(1) + 0.5*stepSize;
    for i=1:1:N
        y_s = map.YWorldLimits(1) + 0.5*stepSize;
        for j=1:1:M
            if (inpolygon(x_s,y_s,map.x,map.y))
                Abs(i,j) = 1;
            end
            y_s = y_s + stepSize;
        end
        x_s = x_s + stepSize;
    end
end
end

