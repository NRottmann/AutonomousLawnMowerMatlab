% This script generates a startup script in order to add these Toolbox to
% your matlab search path. Just run this script once.
%
% Date:     17.05.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

clear all
close all
clc

% Get the folder names
d = dir;
isub = [d(:).isdir];                    % returns logical vector
nameFolds = {d(isub).name}';
nameFolds(ismember(nameFolds,{'.','..','.git'})) = [];

% Get the path of your directory
full = mfilename('fullpath');
name = mfilename;
base = full(1:end-length(name));

% Construct the code which should be inserted into the startup script
for i=1:1:length(nameFolds)
    A{i} = strcat('addpath(''',[base nameFolds{i}],''');');
end
A{length(nameFolds)+1} = strcat('addpath(''',[base],''');');

% Generating the startup script and adding the strings to it
scriptname = sprintf('startup.m');
fid = fopen(scriptname, 'wt');
for i=1:1:length(A)
    fprintf(fid, '%s', A{i});
    fprintf(fid, '\n');
end
fprintf(fid, 'disp(''Lawn Mower Simualtion Environment successfully added to Search Path!'')');
fclose(fid);
edit(scriptname);

% Put the generated startup file into your userpath
movefile('startup.m',userpath);

