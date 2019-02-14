clear all
close all
clc

%% Create polymap for courtyard of the robotic institute
dx = [9.7163, 0.7048, 2.949, -0.7087, 5.8186, 9.6066, 3.1239, -1.3912, 1.62, 1.3697, 5.2688, -18.7929, 0.9425, -3.6167, -0.9314, -15.6772];
dy = [5.257, -1.3076, 1.5815, 1.3063, 3.1731, -2.9304, -5.7703, -0.7454, -2.976, 0.7703, -9.8327, 0.1299, 3.2495, 1.0699, -3.1519, 10.1768];

n = length(dx) - 1;
x = zeros(1,n+2);
y = zeros(1,n+2);
for i=2:1:n+1
    x(i) = x(i-1) + dx(i-1);
    y(i) = y(i-1) + dy(i-1);
end

%% Generate roundings
s1 = [x(6); y(6)]; e1 = [x(7); y(7)];
r1 = 14.2285; n1 = 20;
[x_seg1,y_seg1] = circleSegment(e1,s1,r1,n1,1);
s2 = [x(12); y(12)]; e2 = [x(13); y(13)];
r2 = 39.7766; n2 = 20;
[x_seg2,y_seg2] = circleSegment(e2,s2,r2,n2,2);
s3 = [x(16); y(16)]; e3 = [x(17); y(17)];
r3 = 39.7766; n3 = 20;
[x_seg3,y_seg3] = circleSegment(e3,s3,r3,n3,2);

% Include roundings
x = [x(1:5) x_seg1 x(8:11) x_seg2 x(14:15) x_seg3 0];
y = [y(1:5) y_seg1 y(8:11) y_seg2 y(14:15) y_seg3 0];

% Generate polymap
polyMap = genPolyMap(x,y);
plot(polyMap.x,polyMap.y)
save('map_garden03.mat','polyMap');

% Function description
function [x_seg,y_seg] = circleSegment(e,s,r,n,mode)
    % Generate circle segment
    v = e - s; h = sqrt(r^2 - 0.25*norm(v)^2); v_ = [-v(2); v(1)]/norm(v);
    if mode == 1
        O = s + 0.5*v + h*v_;
    elseif mode == 2
        O = s + 0.5*v - h*v_;
    end
    w1 = s - O; w2 = e - O;
    phi1 = atan2(w1(2),w1(1)); phi2 = atan2(w2(2),w2(1));
    phi = linspace(phi1,phi2,n);
    x_seg = O(1) + cos(phi) * r;
    y_seg = O(2) + sin(phi) * r;
end