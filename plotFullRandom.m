close all, clear all, clc;

times = zeros(20,1);

maxLength = 0;
figure;
for i = 1:1:20
    results = strcat('planned99error90T_', num2str(i));
    results = strcat(results, '.mat');
    load(results);
    
    if length(coverageResults.coverages) > maxLength
        maxLength = length(coverageResults.coverages);
    end
    times(i) = length(coverageResults.coverages)/20;
    plot(coverageResults.coverages);
    hold on
end
mapAbs = mapAbsolute(coverageResults.polyMap, 10);

time = mean(times);
deviation = std(times);
max = max(times);
min = min(times);

coverages = 0.99*ones(16, maxLength);

for i = 1:1:20
    results = strcat('planned99error90T_', num2str(i));
    results = strcat(results, '.mat');
    load(results);
    disp(i)
    for j = 1:1:length(coverageResults.coverages)
        coverages(i, j) = coverageResults.coverages(j);
    end
    coverageResults.groundTruth(mapAbs==0) = 0;
    truecoverages(i) = sum(coverageResults.groundTruth==1)/sum(mapAbs==1);
end
figure;
bar(truecoverages)

schnitt = mean(coverages);
standard = std(coverages);

X = linspace(0, maxLength/20, maxLength);
Y = [schnitt+1*standard; schnitt-1*standard];

figure;
plot(X,schnitt)

figure()
s = shadedErrorBar(X, Y,{@mean,@std},'lineprops','-b','patchSaturation',0.2);
s.mainLine.LineWidth = 1;
