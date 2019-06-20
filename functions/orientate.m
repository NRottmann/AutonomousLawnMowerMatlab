function out = orientate(i,j)
switch i
    case -1
        switch j
            case -1
                out = 5*(pi/4);
            case 0
                out = pi;
            case 1
                out = 3*(pi/4);
        end
    case 0
        switch j
            case -1
                out = 3*(pi/2);
            case 0
                out = inf;
            case 1
                out = (pi/2);
        end
    case 1
        switch j
            case -1
                out = 7*(pi/4);
            case 0
                out = 0;
            case 1
                out = 1*(pi/4);
        end
end
end

