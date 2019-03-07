function pose = generateStartPose(polyMap)
    % Function to generate a random starting pose inside a given polygon
    % map
    % Syntax:
    %       pose = generateStartPose(polyMap)
    % Input:
    %   polyMap         A polygon map in which a starting pose has to be
    %                   chosen
    % Output:   
    %   pose:           A randomly generated starting pose
    while true
        x = rand()*(polyMap.XMapLimits(2) - polyMap.XMapLimits(1)) ...
                    + polyMap.XMapLimits(1);
        y = rand()*(polyMap.YMapLimits(2) - polyMap.YMapLimits(1)) ...
                    + polyMap.YMapLimits(1);
        phi = rand()*2*pi;
        if inpolygon(x,y,polyMap.x,polyMap.y)
            break
        end
    end
    pose = [x; y; phi];     % Initial pose and control signal
end
