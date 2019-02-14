%% Clear everything
close all;
clear all;
clc;

%% Choose the map
map_name = 'map_05.mat';  
load(map_name);

%% Do the simulation
N = 15;
error = zeros(N,1);
for i=1:1:N
    param.compMethod = 3;
    param.T = 3000;
    [results] = simulation(polyMap,3,param);
    error(i) = results.E;
end


%% Evaluation
E{1} = load('E_01.mat');
E{2} = load('E_02.mat');
E{3} = load('E_03.mat');
E{4} = load('E_04.mat');
E{5} = load('E_05.mat');

mu = zeros(5,1);
sigma = zeros(5,1);

for i=1:1:5
    mu(i) = mean(E{i}.error);
    sigma(i) = std(E{i}.error);
end

alpha = 0.1:0.1:0.5;

figure(1)
subplot(1,2,1)
plot(alpha,mu)
set(gca ,'FontSize' ,10) ;
xlabel('$$\alpha$$','Interpreter','latex')
ylabel('$$\mu_{\Delta A}$$','Interpreter','latex')
axis([0 0.6 0 0.3])
box off
subplot(1,2,2)
plot(alpha,sigma)
set(gca ,'FontSize' ,10) ;
xlabel('$$\alpha$$','Interpreter','latex')
ylabel('$$\sigma_{\Delta A}$$','Interpreter','latex')
axis([0 0.6 0 0.1])
box off
