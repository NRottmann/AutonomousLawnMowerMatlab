%% Script for plotting Mapping Results
close all
close all
clc

%% Load data
load('map_6_stdNoise.mat')
iter = 20;

%% Parameter evaluation
c_min = zeros(iter,1);
l_nh = zeros(iter,1);
phi_cycle = zeros(iter,1);
gamma = zeros(iter,2);

for i=1:iter
    c_min(i) = results{i,1}.param.c_min;
    l_nh(i) = results{i,1}.param.l_nh;
    phi_cycle(i) = results{i,1}.param.phi_cycle;
    gamma(i,:) = results{i,1}.param.gamma;
end

%% Map Error
error = zeros(iter,1);
for i=1:iter
    error(i) = results{i,2}.error;
end

figure;
histogram(error,10)