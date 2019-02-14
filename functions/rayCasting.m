function inside = rayCasting(p, polyMap)
    % Checks if the point p lies within the polygon defined by polyMap
    % TODO: Improve this!
    
    x_out = [1000;1000];
    
    inside = true;
    if (p(1) < polyMap.XMapLimits(1))
        if (p(1) > polyMap.XMapLimits(2))
            if (p(2) < polyMap.YMapLimits(1))
                if (p(2) > polyMap.YMapLimits(2))
                    inside = false;
                end
            end
        end
    end
    if (inside)
        % Second if the above was not true
        % Check if the line from x_initial to position crosses an even or odd
        % number of edges. Even number: 
        % Assume P1(x1,y1) = x_initial, P2(x2,y2) = position
        count = 0;
        for j = 1:1:length(polyMap.x(1,:))-1
            x=[p(1) x_out(1) polyMap.x(j) polyMap.x(j+1)];
            y=[p(2) x_out(2) polyMap.y(j) polyMap.y(j+1)];
            dt1=det([1,1,1;x(1),x(2),x(3);y(1),y(2),y(3)]) ...
                        * det([1,1,1;x(1),x(2),x(4);y(1),y(2),y(4)]);
            dt2=det([1,1,1;x(1),x(3),x(4);y(1),y(3),y(4)]) ...
                        * det([1,1,1;x(2),x(3),x(4);y(2),y(3),y(4)]);
            if(dt1<=0 && dt2<=0)
                count = count + 1;         % If lines intesect, count up
            end
        end
        % check if count is an odd number
        if mod(count,2)
            inside = true;
        else
            inside = false;
        end
    end
end