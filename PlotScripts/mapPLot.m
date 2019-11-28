close all, clear all, clc;

map = 'map_NilsOma.mat';  
load(map);
Resolution = 10;
% load('planningExampleGradient.mat')

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

a = sum(sum(ObstacleMap == 1));
b = a/100

figure;
surf(ObstacleMap')

% Initialize Coverage map
% CoverageMap = zeros(N,M);
% 
% figure()
% plot(polyMap.x,polyMap.y, 'lineWidth', 2)
% hold on
% plot(coverageResults.path(1,:), coverageResults.path(2,:))
% % hold on
% % scatter(polyMap.x,polyMap.y, 'r', 'filled')
% % hold off
% xlim([-1 , 16])
% ylim([-3 , 11])
% grid on
% xlabel('x in Meter', 'Interpreter','latex')
% ylabel('y in Meter', 'Interpreter','latex')
% % title('Karte', 'Interpreter','latex')
% % pbaspect([17 14 1])
% pbaspect([1 1 1])
% scalebar
% box off
% axis off
% % figure()
% surf(ObstacleMap')
% grid on
% xlabel('x in Zellen', 'Interpreter','latex')
% ylabel('y in Zellen', 'Interpreter','latex')
% title('Abdeckungsmatrix', 'Interpreter','latex')
% colorbar
% pbaspect([1 1 1])