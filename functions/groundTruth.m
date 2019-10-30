function [idx] = groundTruth(point, polyMap, resolution)
idx = [0,0];
idx(1) = ceil((point(1) - polyMap.XWorldLimits(1)) * resolution);
idx(2) = ceil((point(2) - polyMap.YWorldLimits(1)) * resolution);

end

