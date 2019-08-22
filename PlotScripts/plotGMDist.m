% Script for plotting the general idea of the cost function with the
% Gaussian Mixture Models

clear all
close all
clc

load('Length01.mat');

K = 50;
GMModels = cell(K,1);
Likelihood = zeros(K,1);

for k=1:1:K
    GMModels{k} = fitgmdist(L,k,'RegularizationValue',0.1);
    Likelihood(k) = GMModels{k}.NegativeLogLikelihood;
end

%% Plots
width = 8.2;        % centimeter for a half page site
height = 4.0;

h1 = figure(1);
set(h1, 'Units','centimeters','Position', [1 1 0.3*width height])
k = 1:1:K;
plot(k,Likelihood)
set(gca ,'FontSize' ,10) ;
xlabel('K','Interpreter','latex')
ylabel('$$\mathcal{L}$$','Interpreter','latex')
axis([1 50 0 6000])
box off

h2 = figure(2);
set(h2, 'Units','centimeters','Position', [1 1 0.7*width height])
x = 0:0.1:250;
y = pdf(GMModels{2},x');
histogram(L,170,'normalization','pdf','EdgeAlpha',0.4,'FaceAlpha',0.4)
hold on
plot(x,y)
set(gca ,'FontSize' ,10) ;
xlabel('$$U$$ in meter','Interpreter','latex')
ylabel('$$p(U)$$','Interpreter','latex')
% legend('Histogram','PDF')
% axis([80 220 0 1])   % Dataset 1
axis([0 250 0 1.0]) % Dataset 2
box off
% legend('boxoff')   