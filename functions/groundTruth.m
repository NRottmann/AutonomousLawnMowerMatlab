function A = groundTruth(path, polyMap, resolution)

N = round((polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1)) * resolution);
M = round((polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1)) * resolution);
A = zeros(N, M);
for i = 1:1:size(path, 2)
    idx_x = ceil((path(1, i) - polyMap.XWorldLimits(1)) * resolution);
    idx_y = ceil((path(2, i) - polyMap.YWorldLimits(1)) * resolution);
    A(idx_x, idx_y) = 1;
end
end

