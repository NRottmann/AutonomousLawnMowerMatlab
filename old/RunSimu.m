%% Clear everything
close all;
clear all;
clc;

%% Choose the map
map_name = 'map_garden03.mat';  
load(map_name);

%% Do the simulation
mode = 4;
param.compMethod = 2;
param.T = 3000;
[results] = simulation(polyMap,mode,param);

%% Plot Result
figure(1)
plot(results.polyMap.x, results.polyMap.y)
hold on
plot(results.poseCCPP(1,:),results.poseCCPP(2,:))
plot(results.poseCCPP_est(1,:),results.poseCCPP_est(2,:))










% %% Plot for results section
% % True Shape vs Estimated path
% lineWidth = 1.0;
% figure(1)
% plot(polyMap.x,polyMap.y);
% set(gca ,'FontSize' ,10) ;
% set(findall(gca, 'Type', 'Line'),'LineWidth',lineWidth);
% set(gca, 'Position', [0.01 0.05 0.98 0.9]); 
% obj = scalebar;
% obj.XUnit = 'm';
% obj.YUnit = 'm';
% box off
% axis('off')
% 
% %%
% figure(2)
% subplot(1,2,1)
% plot(results.estPose(1,:),results.estPose(2,:));
% set(gca ,'FontSize' ,10) ;
% set(findall(gca, 'Type', 'Line'),'LineWidth',lineWidth);
% set(gca, 'Position', [0.05 0.05 0.40 0.9]); 
% obj = scalebar;
% obj.XUnit = 'm';
% obj.YUnit = 'm';
% box off
% axis('off')
% subplot(1,2,2)
% plot(polyMap.x,polyMap.y,'k');
% hold on
% plot(results.X_aligned(1,:),results.X_aligned(2,:));
% set(gca ,'FontSize' ,10) ;
% set(findall(gca, 'Type', 'Line'),'LineWidth',lineWidth);
% set(gca, 'Position', [0.55 0.05 0.40 0.9]); 
% obj = scalebar;
% obj.XUnit = 'm';
% obj.YUnit = 'm';
% box off
% axis('off')
% legend('True Shape','Estimate')
% legend boxoff  
% %%
% figure(3)
% subplot(1,4,1)
% plot(results.estPose(1,:),results.estPose(2,:));
% set(gca ,'FontSize' ,10) ;
% % xlabel('x in meter','Interpreter','latex')
% % ylabel('y in meter','Interpreter','latex')
% box off
% subplot(1,4,2)
% plot(results.X(1,:),results.X(2,:));
% set(gca ,'FontSize' ,10) ;
% % xlabel('x in meter','Interpreter','latex')
% % ylabel('y in meter','Interpreter','latex')
% box off
% subplot(1,4,3)
% plot(results.X_cutted(1,:),results.X_cutted(2,:));
% set(gca ,'FontSize' ,10) ;
% % xlabel('x in meter','Interpreter','latex')
% % ylabel('y in meter','Interpreter','latex')
% box off
% subplot(1,4,4)
% plot(results.X_closed(1,:),results.X_closed(2,:));
% set(gca ,'FontSize' ,10) ;
% % xlabel('x in meter','Interpreter','latex')
% % ylabel('y in meter','Interpreter','latex')
% box off
% 
% % %% Additional
% % for i=1:1:5
% %     subplot(1,5,i)
% %     contourf(C{i})
% %     colormap('gray')
% %     set(gca ,'FontSize' ,10) ;
% %     xlabel('x in meter','Interpreter','latex')
% %     ylabel('y in meter','Interpreter','latex')
% %     box off
% % end
