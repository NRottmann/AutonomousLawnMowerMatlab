%% Generate a polygonal map

% % Map 01
% polyMap.x = [0 10 10 6 6 0 0];
% polyMap.y = [0 0 4 4 10 10 0];

% Map 02
% polyMap.x = [0 2 2 4 4 6 6 4 4 3 3 -1 -1 -6 -6 0 0];
% polyMap.y = [0 0 2 2 0 0 6 7 4 4 7 7 6 6 -3 -3 0];

% Map 03
% polyMap.x = [0 30 30 10 10 30 30 0 0];
% polyMap.y = [0 0 10 10 25 25 30 30 0];

% Map 4
% X1 = [0 16 16 10 10 22 22 10];
% Y1 = [0 0 8 8 12 12 18 18];
% N = 20;
% n = 0:1:N;
% X2 = cos(n/N*pi)*5 + 5;
% Y2 = sin(n/N*pi)*5 + 24;
% X3 = [-6 -6 0 0 -6 -6 0 0 -6 -6 0 0];
% Y3 = [24 20 20 16 16 12 12 8 8 4 4 0];
% polyMap.x = [X1 X2 X3];
% polyMap.y = [Y1 Y2 Y3];

% Map 5: apartment floor plan
x = [0 5 5 4 4 5 5 5.5 5.5 8.5 8.5 9.5 9.5 9 9 14 14 10.5 10.5 ...
        14 14 11.5 11.5 11 11 8.5 8.5 15.5 15.5 9 9 8.5 8.5 0 0 1 1 0 0];
y = [0 0 4 4 4.5 4.5 5.5 5.5 0 0 2 2 1.5 1.5 -2.5 -2.5 1.5 1.5 ...
            2 2 4.5 4.5 3 3 4.5 4.5 5 5 10 10 7 7 8.5 8.5 4.5 4.5 4 4 0];


%% Generate poly map and store it
polyMap = genPolyMap(x,y);

plot(polyMap.x,polyMap.y)

save('map_05.mat','polyMap');