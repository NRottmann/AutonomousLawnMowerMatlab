% Create optimal data from scratch, in order to be able to generate also
% DPs we make more then one round
rounds = 1;
nn = 1000;
L = [3; 3+2*pi; 6+2*pi; 9+2*pi; 9+2*pi+sqrt(5); 9+2*pi+2*sqrt(5); ...
        9+2*pi+2*sqrt(5)+3*pi; 11+2*pi+2*sqrt(5)+3*pi; ...
        11+2*pi+2*sqrt(5)+3*pi+sqrt(8); 16+2*pi+2*sqrt(5)+3*pi+sqrt(8)];
l = linspace(0,L(end),nn);
xx = zeros(rounds*nn,1);
yy = zeros(rounds*nn,1);
for j=0:1:rounds-1
    for i=1:1:nn
        if l(i) < L(1)
            xx(i+j*nn) = 0;
            yy(i+j*nn) = l(i);
        elseif l(i) < L(2)
            phi = (l(i)-L(1))/2;
            xx(i+j*nn) = 2*sin(phi);
            yy(i+j*nn) = 5 - 2*cos(phi);
        elseif l(i) < L(3)
            xx(i+j*nn) = 0;
            yy(i+j*nn) = 7 + l(i)-L(2);
        elseif l(i) < L(4)
            xx(i+j*nn) = l(i)-L(3);
            yy(i+j*nn) = 10;
        elseif l(i) < L(5)
            dl = l(i)-L(4);
            xx(i+j*nn) = 3 + (2/sqrt(5))*dl;
            yy(i+j*nn) = 10 - (1/sqrt(5))*dl;
        elseif l(i) < L(6)
            dl = l(i)-L(5);
            xx(i+j*nn) = 5 + (2/sqrt(5))*dl;
            yy(i+j*nn) = 9 + (1/sqrt(5))*dl;
        elseif l(i) < L(7)
            phi = (l(i)-L(6))/3;
            xx(i+j*nn) = 7 + 3*sin(phi);
            yy(i+j*nn) = 7 + 3*cos(phi);
        elseif l(i) < L(8)
            xx(i+j*nn) = 7;
            yy(i+j*nn) = 4 - (l(i)-L(7));
        elseif l(i) < L(9)
            dl = l(i)-L(8);
            xx(i+j*nn) = 7 - (2/sqrt(8))*dl;
            yy(i+j*nn) = 2 - (2/sqrt(8))*dl;
        elseif l(i) < L(10)
            xx(i+j*nn) = 5 - (l(i) - L(9));
            yy(i+j*nn) = 0;
        end
    end
end

polyMap = genPolyMap(xx',yy');
figure
plot(polyMap.x,polyMap.y)

gridMap = genGridMap(polyMap,20);
figure
show(gridMap)

save('map_garden01.mat','polyMap','gridMap');
