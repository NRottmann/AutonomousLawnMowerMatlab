close all, clear all, clc;

map = 'map_01.mat';  
load(map);
Resolution = 10;

% Generate obstacle map, [N,M] Matrix initialized with zeros
% when inside and ones when outside of a the environment
N = round((polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1)) * Resolution);
M = round((polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1)) * Resolution);
ObstacleMap = zeros(N,M);
stepSize = 1/Resolution;
x_s = polyMap.XWorldLimits(1) + 0.5*stepSize;
for i=1:1:N
    y_s = polyMap.YWorldLimits(1) + 0.5*stepSize;
    for j=1:1:M
        if (inpolygon(x_s,y_s,polyMap.x,polyMap.y))
            ObstacleMap(i,j) = 1;
        end
        y_s = y_s + stepSize;
    end
    x_s = x_s + stepSize;
end

% Initialize Coverage map
CoverageMap = zeros(N,M);

figure()
plot(polyMap.x,polyMap.y)
xlim([-1 , 11])
ylim([-1 , 11])
grid on
xlabel('x in Meter')
ylabel('y in Meter')
title('Karte')
pbaspect([1 1 1])
figure()
surf(ObstacleMap')
grid on
xlabel('x in Zellen')
ylabel('y in Zellen')
title('Abdeckungsmatrix')
colorbar
pbaspect([1 1 1])