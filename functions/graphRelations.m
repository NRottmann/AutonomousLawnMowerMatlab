function [results] = graphRelations(DP1,DP2)
% Error measure based on graph relations, see
% "A Comparison of SLAM Algorithms Based on a Graph of Relations"
%
% Right now the graphs are restricted to p = [x,y,phi]
%
% input:
%   DP1:    Dominant points of the first graph
%   DP2:    Dominant points of the second graph
%
% output:
%   results: A structure containing the comparion results
%

% Check if both are equal length
if(length(DP1(:,1)) ~= length(DP2(:,1)))
    error('DPs have not the same dimensions!');
end
if(length(DP1(1,:)) ~= length(DP2(1,:)))
    error('DPs have not the same length!');
end

% Generate relative measurements
xi1 = PoseGraphOptimization.generateMeasurements(DP1);
xi2 = PoseGraphOptimization.generateMeasurements(DP2);

% Get comparative measurements
N = length(xi1(1,:));
xi_comp = zeros(3,N);
for i=1:1:N
    R = [cos(xi2(3,i)) -sin(xi2(3,i)); sin(xi2(3,i)) cos(xi2(3,i))];
    xi_comp(1:2,i) = R' * (xi1(1:2,i) - xi2(1:2,i));
    xi_comp(3,i) = xi1(3,i) - xi2(3,i);
end

% Get the error measurements
errorTrans = 0;
errorRot = 0;
error = 0;
for i=1:1:N
    errorTrans = errorTrans + xi_comp(1:2,i)'*xi_comp(1:2,i);
    errorRot = errorRot + xi_comp(3,i)^2;
    error = error + xi_comp(3,i)^2 + xi_comp(1:2,i)'*xi_comp(1:2,i);
end

results.errorTrans = errorTrans/N;
results.errorRot = errorRot/N;
results.error = error/N;
end

