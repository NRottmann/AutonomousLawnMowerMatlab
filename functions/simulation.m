function [results] = simulation(polyMap,mode,param)
    % This function simulates the robot
    %
    % Syntax:
    %       [results] = simulation(polyMap,mode)
    %
    % Input:
    %   polyMap:        A map of the environment as polygon
    %   Mode:           There are different simulation modes. Please choose
    %                   one of the following
    %                   1: wall following mode, gives back the path
    %                   travelled
    %                   2: mapping mode, we use the wall follower to drive
    %                   along the boundary line and then create a map using
    %                   the presented methods
    %                   3: mapping mode including comparison to the
    %                   original map
    %                   4: covering mode using CCPP methods
    %   param:          
    %       compMethod: Comparison method for map estimate (only required
    %                   if mode 3 chosen), for further information see
    %                   compare2Map
    %
    % Output:
    %   results:        Structure with the results of the simulation. The
    %                   content depends on the chosen mode
    
    if (mode == 1 || mode == 2 || mode == 3 || mode == 4)
        % Initialize classes
        grassSensor = GrassSensor(polyMap);
        odometryModel = OdometryModel();
        wallFollower = WallFollower();
        kinematicModel = KinematicModel();
        randomController = RandomController();
        pdController = PDController();
        % Get sampling time
        out = get_config('system');
        dt =  out.dt;
        % Do the iteration
        T = round(param.T/dt);
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
            [odometryModel,~] = odometryModel.odometryData(truePose(:,i), motionData);
            estPose(:,i+1) = odometryPose(odometryModel, estPose(:,i), true, 1);
        end
        if (mode == 2 || mode == 3 || mode == 4)
            % Pose graph optimization
            poseGraphOptimization = PoseGraphOptimization(estPose(1:2,:));
            [X,A] = generateMap(poseGraphOptimization);
            % Close graph
            [X_cutted,A_cutted] = CutGraph(X,A);
            [X_closed] = CloseGraph(X_cutted,A_cutted);
            % Store results
            results.truePose = truePose;
            results.estPose = estPose;
            results.X = X;
            results.X_cutted = X_cutted;
            results.polyMap = genPolyMap(X_closed(1,:),X_closed(2,:));
            if mode == 3
                % Compare estimated map with the original one 
                [X_aligned,E] = Compare2Map(X_closed,polyMap,param.compMethod);
                results.X_aligned = X_aligned;
                results.E = E;
            elseif mode == 4
                a = 20;             % Passive decay rate
                b = 1.5;            % Upper bound
                d = 1;              % Lower bound
                e_max = 200;       	% Maximum Gain, for gradient
                e_min = 0.5;        % Minimum Gain
                t = 0.8;            % Treshold for when a field counts as covered
                c = 0.1;            % Control gain
                alpha = 0;          % Value for going through
                res = 10;           % Fields per meter
                planner = NNCCPP(results.polyMap,a,b,d,e_max,e_min,t,c,alpha,res);
                  
                symbioticParticleFilter = SymbioticParticleFilter(results.polyMap,[zeros(3,1); 0],grassSensor,...
                            odometryModel,wallFollower,randomController,...
                            pdController,planner,2);
                        
                pose = truePose(:,end);      
                MM = 6000;
                pose_store = zeros(3,MM);
                pose_store_est = zeros(3,MM);
                for i = 1:MM
                    % Step 1: Get sensor measurements
                    [sensorData] = measure(grassSensor,pose);
                    % Step 2: Move Robot and store positions
                    [pose, motionData] = kinModel(kinematicModel, pose, u, true);
                    % Step 3: Corrupt pose with noise
                    [odometryModel,odometryData] = odometryModel.odometryData(pose, motionData);
                    p_corrupted = odometryModel.odometryPose(pose,true,1);
                    % Step 4: Particle Filter algorithm
                    [symbioticParticleFilter,u] = update(symbioticParticleFilter,sensorData,odometryData,p_corrupted);
                    if ~symbioticParticleFilter.GlobalLocalization
                        % Debug
                        figure(10)
                        clf
                        subplot(1,2,1)
                        plot(symbioticParticleFilter.PolyMap.x,symbioticParticleFilter.PolyMap.y)
                        hold on
                        plot(symbioticParticleFilter.Particles(1,:),symbioticParticleFilter.Particles(2,:),'.k')
                        subplot(1,2,2)
                        plot(polyMap.x,polyMap.y)
                        hold on
                        plot(pose(1),pose(2),'*r')
                        drawnow
                    end
                    % Step 5: Store Pose
                    pose_store(:,i) = pose;
                    pose_est_tmp = [mean(symbioticParticleFilter.Particles(1,:)); ...
                                        mean(symbioticParticleFilter.Particles(2,:)); ...
                                       	mean(symbioticParticleFilter.Particles(3,:))];
                    pose_store_est(:,i) = pose_est_tmp;
                end
                results.poseCCPP = pose_store;
                results.poseCCPP_est = pose_store_est;
                results.Filter = symbioticParticleFilter;
                results.Planner = planner;
            end
        else
            results.truePose = truePose;
            results.estPose = estPose;
        end
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