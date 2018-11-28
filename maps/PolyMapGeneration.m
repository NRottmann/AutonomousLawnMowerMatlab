%% Generate a polygonal map

% % Map 01
% polyMap.x = [0 10 10 6 6 0 0];
% polyMap.y = [0 0 4 4 10 10 0];

% Map 02
% polyMap.x = [0 2 2 4 4 6 6 4 4 3 3 -1 -1 -6 -6 0 0];
% polyMap.y = [0 0 2 2 0 0 6 7 4 4 7 7 6 6 -3 -3 0];

% Map 03
polyMap.x = [0 30 30 10 10 30 30 0 0];
polyMap.y = [0 0 10 10 25 25 30 30 0];

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

plot(polyMap.x,polyMap.y)

save('map_03.mat','polyMap');