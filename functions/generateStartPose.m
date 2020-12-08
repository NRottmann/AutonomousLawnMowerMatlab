function pose = generateStartPose(map)
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
        x = rand()*(map.XWorldLimits(2) - map.XWorldLimits(1)) ...
                    + map.XWorldLimits(1);
        y = rand()*(map.YWorldLimits(2) - map.YWorldLimits(1)) ...
                    + map.YWorldLimits(1);
        phi = rand()*2*pi;
        if isa(map,'binaryOccupancyMap')
            if getOccupancy(map,[x,y])
                break
            end
        else
            if inpolygon(x,y,map.x,map.y)
                break
            end
        end
    end
    pose = [x; y; phi];     % Initial pose and control signal
end
