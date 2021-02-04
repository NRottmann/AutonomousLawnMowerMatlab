%% Generate maps
% All maps have the same area of A=50m^2
% The map complexity increases with the number of rectangular corners
% Rules: 0 <= x <= 10, 0 <= y <= 10

clear all
close all
clc

%% Generate the Map Points
map_id = 6;
map_num = 100;

x = cell(map_num,1);
y = cell(map_num,1);

% Map 01, 4 corners
x{1} = [0 5 5 0 0];
y{1} = [0 0 10 10 0];

% Map 02, 6 corners
x{2} = [0 7 7 5 5 0 0];
y{2} = [0 0 5 5 8 8 0];

% Map 03, 8 corners
x{3} = [0 8 8 4 4 7 7 0 0];
y{3} = [0 0 3 3 6 6 8 8 0];

% Map 4, 10 corners
x{4} = [2 9 9 6 6 3 3 0 0 2 2];
y{4} = [0 0 7 7 4 4 7 7 2 2 0];

% Map 5, 12 corners
x{5} = [0 6 6 7 7 9 9 0 0 3 3 0 0];
y{5} = [0 0 6 6 2 2 8 8 6 6 2 2 0];

% Map 6, 14 corners
x{6} = [0 9 9 7 7 5 5 9 9 0 0 3 3 0 0];
y{6} = [0 0 5 5 2 2 6 6 8 8 6 6 2 2 0];


% % Map 10-19: Small to large
% multiplier = [0.5 1 2];
% for i=10:1:12
%     x{i} = multiplier(i-9) * x{9};
%     y{i} = multiplier(i-9) * y{9};
% end

%% Generate poly map and store it
figure
for i=1:6
    
    polyMap = genPolyMap(x{i},y{i});
    gridMap = genGridMap(polyMap,10);
    
    subplot(2,3,i)
    show(gridMap)
    
    s = strcat('map_',num2str(i),'.mat');
    save(s,'polyMap','gridMap');
    
end
