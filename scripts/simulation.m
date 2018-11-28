function [results] = simulation(polyMap,T,mode)
    % This function simulates the robot
    %
    % Syntax:
    %       [results] = simulation(polyMap,mode)
    %
    % Input:
    %   polyMap:        A map of the environment as polygon
    %   Time:           Simulation time in seconds
    %   Mode:           There are different simulation modes. Please choose
    %                   one of the following
    %                   1:  wall following mode, gives back the path
    %                   travelled
    %                   2: TODO
    %
    % Output:
    %   results:        Structure with the results of the simulation. The
    %                   content depends on the chosen mode
    
    if mode == 1
        % Initialize classes
        grassSensor = GrassSensor(polyMap);
        odometryModel = OdometryModel();
        wallFollower = WallFollower();
        kinematicModel = KinematicModel();
        % Get sampling time
        out = get_config('system');
        dt =  out.dt;
        % Do the iteration
        T = round(T/dt);
        truePose = zeros(3,length(T)+1);
        estPose = zeros(3,length(T)+1);
        truePose(:,1) = generateStartPose(polyMap);
        for i = 1:1:T
            % Step 1: Get sensor measurements
            sensorData = measure(grassSensor,truePose(:,i));
            % Step 2: Get control input
            [wallFollower,u] = wallFollowing(wallFollower,sensorData);
            % Step 3: Move Robot and store positions
            [truePose(:,i+1),motionData] = kinModel(kinematicModel, truePose(:,i), u, true);
            % Step 4: Corrupt pose with noise
            [odometryModel,~] = odometryData(odometryModel, truePose(:,i), motionData);
            estPose(:,i+1) = odometryPose(odometryModel, estPose(:,i), true, 1);
        end
        results.truePose = truePose;
        results.estPose = estPose;
    else
        error('simulation: Wrong mode chosen!');
    end
end

function pose = generateStartPose(polyMap)
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