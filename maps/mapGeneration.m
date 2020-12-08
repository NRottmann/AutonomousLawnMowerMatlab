%% Generate a polygonal map

clear all
close all
clc

%% Generate the Map Points
map_id = 8;
map_num = 100;

x = cell(map_num,1);
y = cell(map_num,1);

% Map 01
x{1} = [0 10 10 6 6 0 0];
y{1} = [0 0 4 4 10 10 0];

% Map 02
x{2} = [0 2 2 4 4 6 6 4 4 3 3 -1 -1 -6 -6 0 0];
y{2} = [0 0 2 2 0 0 6 7 4 4 7 7 6 6 -3 -3 0];

% Map 03
x{3} = [0 30 30 10 10 30 30 0 0];
y{3} = [0 0 10 10 25 25 30 30 0];

% Map 4
X1 = [0 16 16 10 10 22 22 10];
Y1 = [0 0 8 8 12 12 18 18];
N = 20;
n = 0:1:N;
X2 = cos(n/N*pi)*5 + 5;
Y2 = sin(n/N*pi)*5 + 24;
X3 = [-6 -6 0 0 -6 -6 0 0 -6 -6 0 0];
Y3 = [24 20 20 16 16 12 12 8 8 4 4 0];
x{4} = [X1 X2 X3];
y{4} = [Y1 Y2 Y3];

% Map 5: apartment floor plan
x{5} = [0 5 5 4 4 5 5 5.5 5.5 8.5 8.5 9.5 9.5 9 9 14 14 10.5 10.5 ...
        14 14 11.5 11.5 11 11 8.5 8.5 15.5 15.5 9 9 8.5 8.5 0 0 1 1 0 0];
y{5} = [0 0 4 4 4.5 4.5 5.5 5.5 0 0 2 2 1.5 1.5 -2.5 -2.5 1.5 1.5 ...
            2 2 4.5 4.5 3 3 4.5 4.5 5 5 10 10 7 7 8.5 8.5 4.5 4.5 4 4 0];
        
% Map 6: Curved Map
load('curved_map.mat');
X = table2array(curvedmap);
X = [X; X(1,:)];
x{6} = X(:,1)';
y{6} = X(:,2)';

% Map 7: Recurrent Structures
x{7} = [0 3 3 3.5 3.5 5.5 5.5 6 6 9 9 6 6 5.5 5.5 6 6 9 9 6 6 5.5 5.5 3.5 3.5 3 3 0 0 3 3 3.5 3.5 3 3 0 0];
y{7} = [0 0 1.5 1.5 0 0 1.5 1.5 0 0 5 5 3.5 3.5 7 7 5.5 5.5 10.5 10.5 9 9 10.5 10.5 9 9 10.5 10.5 5.5 5.5 7 7 3.5 3.5 5 5 0];
% x = [0 5 5 4 4 5 5 5.5 5.5 8.5 8.5 9.5 9.5 9 9 14 14 10.5 10.5 ...
%         14 14 11.5 11.5 11 11 8.5 8.5 15.5 15.5 9 9 8.5 8.5 0 0 1 1 0 0];
% y = [0 0 4 4 4.5 4.5 5.5 5.5 0 0 2 2 1.5 1.5 -2.5 -2.5 1.5 1.5 ...
%             2 2 4.5 4.5 3 3 4.5 4.5 5 5 10 10 7 7 8.5 8.5 4.5 4.5 4 4 0];

% Map 8: Real Garden (Nils Oma)
x{8} = [0 21 21 17 14 14 15 15 14 14 5 5 3 3 5 5 0 0];
y{8} = [0 0 21 21 17 14 13 9 7 6 6 8 8 11 11 17 17 0];

% Map 10-19: Small to large
multiplier = linspace(1,10,10);
for i=10:1:19
    x{i} = multiplier(i-9) * x{1};
    y{i} = multiplier(i-9) * y{1};
end

%% Generate poly map and store it
polyMap = genPolyMap(x{map_id},y{map_id});
figure
plot(polyMap.x,polyMap.y)

gridMap = genGridMap(polyMap,20);
figure
show(gridMap)

s = strcat('map_',num2str(map_id),'.mat');
save(s,'polyMap','gridMap');
