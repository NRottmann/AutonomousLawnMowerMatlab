function [Abs] = mapAbsolute(polyMap, Resolution)
N = round((polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1)) * Resolution);
M = round((polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1)) * Resolution);
Abs = zeros(N,M);
stepSize = 1/Resolution;
x_s = polyMap.XWorldLimits(1) + 0.5*stepSize;
for i=1:1:N
    y_s = polyMap.YWorldLimits(1) + 0.5*stepSize;
    for j=1:1:M
        if (inpolygon(x_s,y_s,polyMap.x,polyMap.y))
            Abs(i,j) = 1;
        end
        y_s = y_s + stepSize;
    end
    x_s = x_s + stepSize;
end
end

