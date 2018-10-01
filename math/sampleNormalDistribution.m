function [s] = sampleNormalDistribution(a)
% Sampling algorithm for a zero mean normal dsitribution with variance a
%
% Syntax:
%       [s] = sampleNormalDistribution(a)
%
% Input:
%   a:  variance
%
% Output:
%   s:  sample
%
% Date:     01.10.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

s = (a/6) * sum(2*rand(10,1) - 1);

end

