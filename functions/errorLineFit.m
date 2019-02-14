function e = errorLineFit(X)
    % This functions calculates the line error between points
    % of the data set X. Therefore we assume there is a line
    % between the first and the last point of the given data
    % set.
    %
    % Syntax:
    %       e = errorLineFit(X)
    %
    % input:
    %   X:   	Data set with points (2 x K matrix)
    % output:
    %   e:      Error between line and points

    x0 = X(:,1);
    x1 = X(:,end);
    v = x1 - x0;
    phi = atan2(v(2),v(1));
    m = length(X(1,:)) - 1;
    e = 0;
    for j=2:1:m
        vTest = X(:,j) - x0;
        phiTest = atan2(vTest(2),vTest(1));
        psi = phiTest - phi;
        e = e + (sin(psi)*norm(vTest))^2;
    end
    e = e/(m-1);
end
