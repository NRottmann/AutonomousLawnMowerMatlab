clear all, close all, clc;

maxLength = 0;

for i = 1:1:10
    results = strcat('random_', num2str(i));
    results = strcat(results, '.mat');

    load(results);
    
    if length(coverageResults.path) > maxLength
        maxLength = length(coverageResults.path);
    end
end

errorsCoverage = zeros(10,maxLength);
errorsParticle = zeros(10,maxLength);

coveragesInd = zeros(10, 900);

for i = 1:1:10
    disp(i)
    results = strcat('random_', num2str(i));
    results = strcat(results, '.mat');

    load(results);
    
    for j = 1:1:size(coverageResults.errors, 2)
        errorsCoverage(i,j) = coverageResults.errors(1,j);
        errorsParticle(i,j) = coverageResults.errors(2,j);
    end
    
    for j = 1:1:900
        coveragesInd(i,j) = max(find(coverageResults.coverages < j/1000));
    end
end

midErrorsCoverage = zeros(10,900);
midErrorsParticle = zeros(10,900);

for i = 1:1:10
    disp(i)
    
    for j = 2:1:900
        sumCoverage = 0;
        sumParticle = 0;
        inc = 0;
        for k = coveragesInd(i,j-1):1:coveragesInd(i,j)
            inc = inc + 1;
            sumCoverage = sumCoverage + errorsCoverage(i, k);
            sumParticle = sumParticle + errorsParticle(i, k);
        end
        midErrorsCoverage(i,j) = sumCoverage/inc;
        midErrorsParticle(i,j) = sumParticle/inc;
    end
end

errorCoverage = sum(midErrorsCoverage)/10;
errorParticle = sum(midErrorsParticle)/10;

varsCoverage = std(midErrorsCoverage);
varsParticle = std(midErrorsParticle);

X = linspace(0,0.9,900);

% figure()
% plot(X, errorCoverage, 'b', 'lineWidth', 2)
% hold on
% plot(X, errorParticle, 'r', 'lineWidth', 2)
% legend('Mittelwert basierend', 'Partikel basierend', 'Location', 'northwest')
% xlabel('prozentuale Abdeckung')
% ylabel('mse')
% hold off

YCoverage = [errorCoverage+1*varsCoverage; errorCoverage-1*varsCoverage];
YParticle = [errorParticle+1*varsParticle; errorParticle-1*varsParticle];

h = figure()
s = shadedErrorBar(X, YCoverage,{@mean,@std},'lineprops','-b','patchSaturation',0.2);
s.mainLine.LineWidth = 1;
hold on
s = shadedErrorBar(X, YParticle,{@mean,@std},'lineprops','-r','patchSaturation',0.2);
s.mainLine.LineWidth = 1;
legend('Mittelwert basierend', 'Partikel basierend', 'Location', 'northwest', 'Interpreter','latex')
legend('boxoff')
xlabel('prozentuale Abdeckung', 'Interpreter','latex')
ylabel('MSE', 'Interpreter','latex')
hold off
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'verwinkeltpdf','-dpdf','-r0')















