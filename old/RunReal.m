%% Clear everything
close all;
clear all;
clc;

%% Choose the data set
load('Garden03.mat')
load('map_garden03.mat')

%% Do the simulation
param.map = polyMap;
param.compMethod = 1;
[results] = realData(pose,2,param);

%% Plot
figure(1)
subplot(1,2,1)
plot(polyMap.x,polyMap.y);
set(gca ,'FontSize' ,10) ;
xlabel('x in meter','Interpreter','latex')
ylabel('y in meter','Interpreter','latex')
box off
subplot(1,2,2)
plot(pose(1,:),pose(2,:));
set(gca ,'FontSize' ,10) ;
xlabel('x in meter','Interpreter','latex')
ylabel('y in meter','Interpreter','latex')
box off

%% Plots
lineWidth = 1.0;
figure(2)
subplot(1,2,1)
plot(pose(1,:),pose(2,:));
set(gca ,'FontSize' ,10) ;
set(findall(gca, 'Type', 'Line'),'LineWidth',lineWidth);
set(gca, 'Position', [0.01 0.05 0.48 0.9]); 
obj = scalebar;
obj.XUnit = 'm';
obj.YUnit = 'm';
box off
axis('off')
subplot(1,2,2)
plot(polyMap.x,polyMap.y,'k');
hold on
plot(results.X_aligned(1,:),results.X_aligned(2,:));
set(gca ,'FontSize' ,10) ;
set(findall(gca, 'Type', 'Line'),'LineWidth',lineWidth);
set(gca, 'Position', [0.51 0.05 0.48 0.9]); 
obj = scalebar;
obj.XUnit = 'm';
obj.YUnit = 'm';
box off
axis('off')
legend('True Shape','Estimate')
legend boxoff  

%%
figure(3)
subplot(1,4,1)
plot(pose(1,:),pose(2,:));
set(gca ,'FontSize' ,10) ;
% xlabel('x in meter','Interpreter','latex')
% ylabel('y in meter','Interpreter','latex')
box off
subplot(1,4,2)
plot(results.X(1,:),results.X(2,:));
set(gca ,'FontSize' ,10) ;
% xlabel('x in meter','Interpreter','latex')
% ylabel('y in meter','Interpreter','latex')
box off
subplot(1,4,3)
plot(results.X_cutted(1,:),results.X_cutted(2,:));
set(gca ,'FontSize' ,10) ;
% xlabel('x in meter','Interpreter','latex')
% ylabel('y in meter','Interpreter','latex')
box off
subplot(1,4,4)
plot(results.X_closed(1,:),results.X_closed(2,:));
set(gca ,'FontSize' ,10) ;
% xlabel('x in meter','Interpreter','latex')
% ylabel('y in meter','Interpreter','latex')
box off

%%
figure(4)
subplot(1,3,1)
plot(results.X(1,:),results.X(2,:));
set(findall(gca, 'Type', 'Line'),'LineWidth',1.5);
set(gca, 'Position', [0.01 0.05 0.33 0.9]); 
obj = scalebar;
obj.XUnit = 'm';
obj.YUnit = 'm';
box off
axis('off')

subplot(1,3,2)
plot(results.X_cutted(1,:),results.X_cutted(2,:));
set(gca, 'Position', [0.34 0.1 0.33 0.8]); 
box off
axis('off')
subplot(1,3,3)
plot(results.X_closed(1,:),results.X_closed(2,:));
set(gca, 'Position', [0.67 0.1 0.33 0.8]); 
box off
axis('off')