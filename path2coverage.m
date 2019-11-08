function [coverage] = path2coverage(path, mapAbs, polyMap, maxPlan)
coverage = zeros(1,maxPlan);
N = round((polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1)) * 10);
M = round((polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1)) * 10);
ground = zeros(N,M);
for i=1:1:length(path)
    v = groundTruth(path(:,i), polyMap, 10);
    vx = v(1);
    vy = v(2);
    if ((vx>=1 && vx<=N) && (vy>=1 && vy<=M))
        ground(vx, vy) = 1;
    end
    coverage(i) = sum(ground == 1)/sum(mapAbs==1);
end
coverage(coverage==0) = max(coverage);
end

