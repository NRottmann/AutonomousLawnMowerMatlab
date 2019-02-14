function theta = getOrientation(phi,l,x)
    % Get the orientation based on the cumulative length and cumulative
    % orientation
    %
    % input:
    %   phi:    Cumulative orientation
    %   l:      Cumulative length
    %   x:      Evalutation position
    % output:
    %   theta:  The orientation at position x

    theta = 0;
    for i=1:1:length(l)-1
        if l(i) < x
            theta = phi(i+1);
        end
    end
end


