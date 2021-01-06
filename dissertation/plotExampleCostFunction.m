% Plot an examplary cost function
close all
clear all
clc

width = (3/4) * 16.2;         % centimeter for a page site
height = 6.0;

load('data/exampleCostFunction.mat')
h1 = figure;
set(h1, 'Units','centimeters','Position', [1 1 width height])
surf(XX,YY,cost_plot_tmp)
set(gca,'YScale','log')
set(gca ,'FontSize' ,10)
hold on
[~,h2] = contourf(XX,YY,cost_plot_tmp,50,'LineStyle','none');
h2.ContourZLevel = -9;

xlabel('$$l_{nh}$$','Interpreter','latex')
ylabel('$$c_{max}$$','Interpreter','latex')
zlabel('$$c_1(l_{nh},c_{max})$$','Interpreter','latex')

zticks([-2 0 2 4])
box off
