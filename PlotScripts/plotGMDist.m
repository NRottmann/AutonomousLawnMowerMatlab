clear all
close all
clc

load('Length01.mat');

K = 10;
GMModels = cell(K,1);
Likelihood = zeros(K,1);

for k=1:1:K
    GMModels{k} = fitgmdist(L,k,'RegularizationValue',0.1);
    Likelihood(k) = GMModels{k}.NegativeLogLikelihood;
end

%% Plots
figure(1)
k = 1:1:K;
plot(k,Likelihood)
set(gca ,'FontSize' ,10) ;
xlabel('K','Interpreter','latex')
ylabel('$$\mathcal{L}$$','Interpreter','latex')

figure(2)
x = 0:0.1:250;
y = pdf(GMModels{2},x');
histogram(L,100,'normalization','pdf','EdgeAlpha',0.4,'FaceAlpha',0.4)
hold on
plot(x,y)
set(gca ,'FontSize' ,10) ;
xlabel('$$U$$ in meter','Interpreter','latex')
ylabel('$$p(U)$$','Interpreter','latex')
legend('Histogram','PDF')