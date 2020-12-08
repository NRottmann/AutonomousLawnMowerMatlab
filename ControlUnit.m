classdef ControlUnit
    % Control unit for the lawn mower
    %
    % Methods:
    %   ControlUnit(polyMap)
    %       Constructior, initializes all classes with a given polyMap
    %
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 06.03.2019
    
    properties
        %% Classes
        % Planning
        ClassNNCCPP;
        ClassCoverage;
        % Models
        ClassOdometryModel;
        ClassKinematicModel;
        ClassGrassSensor;
        % Mapping
        ClassPoseGraphOptimization;
        ClassMapPostProcessing;
        % Localization
        ClassGlobalLocalizer
        ClassParticleFilter;
        % Control
        ClassWallFollower;
        ClassRandomController;
        ClassPDController;
        
        %% Storage Variables
        Map;
        EstMap;
        Pose;
        EstPose;
        Resolution;
        
        %% Parameters
        Dt;
        Threshhold;
        WallFollow;
    end
    
    methods
        function obj = ControlUnit(map,pose)
            % This is the constructor of the class
            % Syntax:
            %       obj = ControlUnit()
            % Input:
            %   polyMap:     	The map as polygon
            %   pose:           Starting pose of the robot
            % Output:
            %   obj:            Intance of the class ControlUnit
            
            %% Initialize the classes
            % Models
            obj.ClassOdometryModel = OdometryModel();
            obj.ClassKinematicModel = KinematicModel();
            obj.ClassGrassSensor = GrassSensor(map);
            
            % Controller
            obj.ClassWallFollower = WallFollower();
            obj.ClassRandomController = RandomController();
            obj.ClassPDController = PDController();
            
            % Mapping
            obj.ClassPoseGraphOptimization = PoseGraphOptimization();
            obj.ClassMapPostProcessing = MapPostProcessing();
            
            % Localization
            obj.ClassGlobalLocalizer = GlobalLocalizer(map);
            
            % Planning
            obj.ClassCoverage = Coverage();
            obj.ClassNNCCPP = NNCCPP();
            
            %% Initialize variables
            obj.Map = map;
            obj.EstMap = map;
            obj.Pose = pose;
            obj.EstPose = pose;
            
            %% Get Parameters
            out = get_config('system');
            obj.Dt =  out.dt;
            
            out = get_config('coverageMap');
            obj.Resolution = out.resolution;
            obj.Threshhold = out.threshhold;
            obj.WallFollow = out.wallFollow;
        end
        
        function [obj,path,estPath] = wallFollowing(obj,T,mode)
            % This is the wall following simulator
            % Syntax:
            %       [path,estPath] = wallFollowing(obj,T)
            % Input:
            %   T:              Simulation time
            %   mode:           Choose either 0 for random start pose or 1
            %                   for defined starting pose
            % Output:
            %   path:           True path travelled
            %   estPath:        Estimated path travelled
            
            I = round(T/obj.Dt);
            path = zeros(3,I+1);
            estPath = zeros(3,I+1);
            if mode == 0
                path(:,1) = generateStartPose(obj.Map);     % Start with random initial start pose
            elseif mode == 1
                path(:,1) = obj.Pose;
            else
                error('Wrong mode chosen!')
            end
            for i = 1:1:I
                % Step 1: Get sensor measurements
                sensorData = obj.ClassGrassSensor.measure(path(:,i));
                % Step 2: Get control input
                [obj.ClassWallFollower,u] = obj.ClassWallFollower.wallFollowing(sensorData, 0); % right sensor
                % Step 3: Move Robot and store positions
                [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                % Step 4: Corrupt pose with noise
                [obj.ClassOdometryModel,~] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                estPath(:,i+1) = obj.ClassOdometryModel.odometryPose(estPath(:,i), true, 1);
            end
            obj.Pose = path(:,end);
            obj.EstPose = estPath(:,end);
        end
        
        function [obj,results] = mapping(obj,path,optimize,mode)
            % This is the mapping mode
            % Syntax:
            %       results = mapping(obj,path)
            % Input:
            %   path:           Path data used for generating the map
            %   optimize:   struct for optimization
            %       loopClosure:    if true, we optimize l_nh and c_max
            %       mapping:        if true, we optimize gamma1 and gamma2
            % Output:
            %   results:        Results of the mapping approach
            %       optimizedPath:  The whole path after optimization
            %       cuttedPath:     path with cutted edges
            %       closedPath:     Closed path
            %       polyMap:        The map estimate
            
            % Generate optimized path data
            [obj.ClassPoseGraphOptimization,path,A,Circumference] = obj.ClassPoseGraphOptimization.generateMap(path(1:2,:),optimize,mode);
            obj.ClassMapPostProcessing.DP = path(1:2,:);
            obj.ClassMapPostProcessing.A = A;
            obj.ClassMapPostProcessing.Circumference = Circumference;
            % Cut ends and close the graph
            obj.ClassMapPostProcessing = obj.ClassMapPostProcessing.cutGraph();
            obj.ClassMapPostProcessing = obj.ClassMapPostProcessing.closeGraph();
            % Generate Map
            if (isa(obj.Map,'binaryOccupancyMap'))
                obj.ClassMapPostProcessing = obj.ClassMapPostProcessing.generateGridMap();
            else
                obj.ClassMapPostProcessing = obj.ClassMapPostProcessing.generatePolyMap();
            end
            % Adjust estimated map
            obj.EstMap = obj.ClassMapPostProcessing.EstMap;
            % Allocate results
            results.estMap = obj.ClassMapPostProcessing.EstMap;      
            results.DP = obj.ClassMapPostProcessing.DP;
            results.DP_indices = obj.ClassPoseGraphOptimization.DP_indices;
            results.cutDP = obj.ClassMapPostProcessing.CutDP;
            results.cut_indices = obj.ClassMapPostProcessing.Cut_indices;
            results.closedDP = obj.ClassMapPostProcessing.ClosedDP;       
            results.A = obj.ClassMapPostProcessing.A;             
            results.cutA = obj.ClassMapPostProcessing.CutA;
            results.param.gamma(1,1) = obj.ClassPoseGraphOptimization.Gamma1;
            results.param.gamma(1,2) = obj.ClassPoseGraphOptimization.Gamma2;
            results.param.beta(1,1) = obj.ClassPoseGraphOptimization.Beta1;
            results.param.beta(1,2) = obj.ClassPoseGraphOptimization.Beta2;
            results.param.beta(1,3) = obj.ClassPoseGraphOptimization.Beta3;
            results.param.beta(1,4) = obj.ClassPoseGraphOptimization.Beta4;
            results.param.l_min = obj.ClassPoseGraphOptimization.L_min;
            results.param.e_max = obj.ClassPoseGraphOptimization.E_max;
            results.param.l_nh = obj.ClassPoseGraphOptimization.L_nh;
            results.param.c_max = obj.ClassPoseGraphOptimization.C_max;
            results.param.phi_cycle = obj.ClassPoseGraphOptimization.Phi_cycle;
            results.param.m = obj.ClassPoseGraphOptimization.M; 
        end

        function [obj,results] = compare(obj,mode)
            % This is the compare method for the 
            % Syntax:
            %       results = compare(obj,path)
            % Input:
            %   mode            Comparison method
            % Output:
            %   results:        Results of the mapping approach
            %       error:          The error between the true map and the
            %                       estimated one
            %       alignedPath:    The aligned path of the map estimate
            
            results = obj.ClassMapPostProcessing.compareMaps(obj.PolyMap,mode);
            obj.EstPolyMap = results.turnedEstPolyMap;         
        end
        
        function [obj,results] = globalLocalization(obj,T,mode,mapParam)
            % This is the method for global localization
            % Syntax:
            %       [obj,results] = globalLocalization(obj)
            % Input:
            %   T:              Maximum time used for global localization
            %                   before stopping
            %   mode:           Mode used
            %                   1: Random starting position within the map
            %                   2: Current position as starting position
            %   mapParam:       If true, we use the parameter from the pose
            %                   graph optimization, else the in the config
            %                   defined parameter
            % Output:   
            %   results:        Results of the localization approach
            %
            %  
            % Get parameter if required
            if mapParam
                obj.ClassGlobalLocalizer.C_min = obj.ClassPoseGraphOptimization.C_min;
                obj.ClassGlobalLocalizer.L_nh = obj.ClassPoseGraphOptimization.L_nh;
            else
                out = get_config('globalLocalization');
                obj.ClassGlobalLocalizer.C_min = out.c_min;
                obj.ClassGlobalLocalizer.L_nh = out.l_nh;
            end
            % Allocate actual map estimate
            obj.ClassGlobalLocalizer.PolyMap = obj.EstPolyMap;
            % Set wall follower back to find the wall first
            % TODO: Make the wall follower moe intelligent, such that
            % inidcates by itself whether it has to search for a wall
            obj.ClassWallFollower.Mode = 0;
            % Get maximum number of iterations
            I = round(T/obj.Dt);
            % Initialize storage capacities and allocate pose
            path = zeros(3,I+1);
            estPath = zeros(3,I+1);
            % Choose real starting pose depending on the mode
            if mode == 1
                obj.Pose = generateStartPose(obj.PolyMap);
            elseif mode == 2
                obj.Pose = obj.Pose;
            else
                error('ControlUnit.GlobalLocalization: Wrong mode chosen!')
            end
            path(:,1) = obj.Pose;
            % Initialize parameters
            i = 1;
            results.foundPosition = false;
            while ~results.foundPosition && i < I
                % Step 1: Get sensor measurements
                sensorData = obj.ClassGrassSensor.measure(path(:,i));
                % Step 2: Get control input
                [obj.ClassWallFollower,u] = obj.ClassWallFollower.wallFollowing(sensorData);
                % Step 3: Move Robot and store positions
                [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                % Step 4: Corrupt pose with noise
                [obj.ClassOdometryModel,~] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                estPath(:,i+1) = obj.ClassOdometryModel.odometryPose(estPath(:,i), true, 1);
                % Step 5: Use Global Localization
                if obj.ClassWallFollower.Mode == 0
                    obj.ClassGlobalLocalizer.DP = estPath(1:2,i+1);
                    obj.ClassGlobalLocalizer.S = estPath(1:2,i+1);
                elseif obj.ClassWallFollower.Mode == 1
                    [obj.ClassGlobalLocalizer,results] = obj.ClassGlobalLocalizer.localization(estPath(:,i+1));
                else
                    error('Wrong mode!');
                end
                % Count up
                i = i + 1;
            end
            results.path = path;
            results.estPath = estPath;
            results.pose = path(:,i);
            obj.Pose = path(:,i);
            if results.foundPosition
                obj.EstPose = results.estPose;
            else
                disp('Did not find a pose match for pose estimate!')
                obj.EstPose = [0;0;0];
            end
        end
        
        function [obj,results] = completeCoverage(obj,T,mode,particleMap)
            % This is the method is for complete coverage of the workspace
            % Syntax:
            %       [obj,results] = globalLocalization(obj)
            % Input:
            %   T:              Maximum time used for CCPP
            %                   before stopping
            %   mode:           Coverage mode
            %       1:          Random Walk
            %       2:          NNCCPP
            % Output:
            %   results:        Results of the CCPP approach
            %
            
            % Allocate actual map estimate
            obj.ClassParticleFilter = ParticleFilter(obj.EstPolyMap);
            % Get maximum number of iterations
            I = round(T/obj.Dt);
            % Initialize storage capacities and allocate pose
            path = zeros(3,I+1);
            path(:,1) = obj.Pose;
            estPath = path;
            mapAbs = mapAbsolute(obj.PolyMap, obj.Resolution);
            N = round((obj.PolyMap.XWorldLimits(2) - obj.PolyMap.XWorldLimits(1)) * obj.Resolution);
            M = round((obj.PolyMap.YWorldLimits(2) - obj.PolyMap.YWorldLimits(1)) * obj.Resolution);
            ground = zeros(N, M);
            % Decide which mode
            if mode == 1
                % Initialize
                odometryData.DeltaR1 = 0; odometryData.DeltaT = 0; odometryData.DeltaR2 = 0;
                % TODO: Add here choices
                obj.ClassParticleFilter = obj.ClassParticleFilter.initializeParticles(path(:,1),3);
                obj.ClassCoverage = obj.ClassCoverage.initializeCoverageMap(obj.EstPolyMap);
                coverages = zeros(I, 1);
                figure;
                for i=1:1:I
                    disp(i*obj.Dt)
                    % Step 1: Get sensor measurements
                    sensorData = obj.ClassGrassSensor.measure(path(:,i));
                    % Step 2: Get control input
                    [obj.ClassRandomController,u] = obj.ClassRandomController.randomControl(sensorData,odometryData);
                    % Step 3: Move Robot and store positions
                    [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                    % Step 4: Corrupt pose with noise
                    [obj.ClassOdometryModel,odometryData] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                    % Step 5: Use Particle Filter
                    obj.ClassParticleFilter = obj.ClassParticleFilter.updateParticles(sensorData,odometryData, false, true);
                    [estPath(:,i+1),~] = obj.ClassParticleFilter.getPoseEstimate();
                    spread = obj.ClassParticleFilter.getParticleSpread()
                    % Step 6: Update Coverage Map
                    obj.ClassCoverage = obj.ClassCoverage.updateCoverageMap(obj.ClassParticleFilter.Particles,estPath(:,i+1));
                    % Coverage
                    if i == 10
                        scatter(obj.ClassParticleFilter.Particles(1,:),obj.ClassParticleFilter.Particles(2,:),'.b');
                        hold on
                    end
                end
                scatter(obj.ClassParticleFilter.Particles(1,:),obj.ClassParticleFilter.Particles(2,:),'.r');
                plot(path(1,:),path(2,:),'LineWidth',2,'Color','#EDB120')
                set(gca,'visible','off')
                scalebar
                
            elseif mode == 3
                p = T;
                if T >=1
                    p = 1;
                end
                % Initialize
                odometryData.DeltaR1 = 0; odometryData.DeltaT = 0; odometryData.DeltaR2 = 0; coverage = 0; i = 0;
                % TODO: Add here choices
                obj.ClassParticleFilter = obj.ClassParticleFilter.initializeParticles(path(:,1),3);
                obj.ClassCoverage = obj.ClassCoverage.initializeCoverageMap(obj.EstPolyMap);
                while coverage < p
                    i = i + 1;
                    disp(i*obj.Dt)
                    % Step 1: Get sensor measurements
                    sensorData = obj.ClassGrassSensor.measure(path(:,i));
                    % Step 2: Get control input
                    [obj.ClassRandomController,u] = obj.ClassRandomController.randomControl(sensorData,odometryData);
                    % Step 3: Move Robot and store positions
                    [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                    % Step 4: Corrupt pose with noise
                    [obj.ClassOdometryModel,odometryData] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                    % Step 5: Use Particle Filter
                    obj.ClassParticleFilter = obj.ClassParticleFilter.updateParticles(sensorData,odometryData, false, true);
                    [estPath(:,i+1),~] = obj.ClassParticleFilter.getPoseEstimate();
                    % Step 6: Update Coverage Map
                    obj.ClassCoverage = obj.ClassCoverage.updateCoverageMap(obj.ClassParticleFilter.Particles,estPath(:,i+1));
                    % Coverage
                    v = groundTruth(path(:,i), obj.PolyMap, obj.Resolution);
                    vx = v(1);
                    vy = v(2);
                    if ((vx>=1 && vx<=N) && (vy>=1 && vy<=M))
                        ground(vx, vy) = 1;
                    end
                    ground(mapAbs==0) = 0;
                    coverage = sum(ground>=obj.Threshhold)/sum(mapAbs==1);
                    disp(coverage);
                    coverages(i) = coverage;
                end
                figure()
                plot(coverages);
            elseif mode == 2
                % TODO: Add here choices
                obj.ClassParticleFilter = obj.ClassParticleFilter.initializeParticles(path(:,1),3);
                obj.ClassCoverage = obj.ClassCoverage.initializeCoverageMap(obj.EstPolyMap);
                obj.ClassNNCCPP = obj.ClassNNCCPP.initializeNeuralNet(obj.EstPolyMap);
                spread = 0;
                relocating = false;
                spreads = zeros(I, 1);
                coverages = zeros(I, 1);
                for i=1:1:I
                    disp(i*obj.Dt)
                    % Step 1: Get sensor measurements
                    sensorData = obj.ClassGrassSensor.measure(path(:,i));
                    % Step 2: Get control input with state machine
                    if spread > obj.WallFollow && ~relocating
                        relocating = true; % change to wallfollowing
                        if coverage < 0.5 % half the map not covered
                            half = false;
                            if ((mod(estPath(3,i), 2*pi) > (3*pi)/2) || (mod(estPath(3,i), 2*pi) < pi/2))
                                sensor = 1; % right sensor
                            else
                                sensor = 0; % left sensor
                            end
                        else % half the map already covered
                            half = true;
                            if ((mod(estPath(3,i), 2*pi) > (3*pi)/2 || mod(estPath(3,i), 2*pi) < pi/2))
                                sensor = 0; % left sensor
                            else
                                sensor = 1; % right sensor
                            end
                        end
                    end
                    if (relocating && spread < obj.WallFollow/3) % change from wallfollowing to NNCCPP
                        idx_x = ceil((estPath(1,i) - obj.PolyMap.XWorldLimits(1)) * obj.Resolution);
                        idx_y = ceil((estPath(2,i) - obj.PolyMap.YWorldLimits(1)) * obj.Resolution);
                        if ((~half && obj.ClassNNCCPP.ExternalInput(idx_x,idx_y) ~= 0) || (half && obj.ClassNNCCPP.ExternalInput(idx_x,idx_y) == 0))
                            relocating = false;
                            obj.ClassWallFollower = WallFollower();
                        end
                    end
                    if relocating % wallfollow
                        [obj.ClassWallFollower,u] = obj.ClassWallFollower.wallFollowing(sensorData, sensor);
                    else % netting
                        orientation = mod(estPath(3,i), 2*pi);
                        % allocate control weight based on orientation
                        if (orientation > 7.5*pi/4 || orientation < 0.5*pi/4) || (orientation > 3.5*pi/4 && orientation < 4.5*pi/4)
                            obj.ClassNNCCPP.C = 100;
                        else
                            obj.ClassNNCCPP.C = 1.5;
                        end
                        if particleMap % use particlemap
                            [obj.ClassNNCCPP,x] = obj.ClassNNCCPP.planStep(estPath(:,i),obj.ClassCoverage.CoverageMap);
                        else % use coveragemap
                            [obj.ClassNNCCPP,x] = obj.ClassNNCCPP.planStep(estPath(:,i),obj.ClassCoverage.CoverageMap);
                        end
                        % get movement data
                        [obj.ClassPDController,u] = obj.ClassPDController.pdControl(estPath(1:2,i),[0;0],x,[0;0],estPath(3,i),0);
                    end
                    % Step 3: Move Robot and store positions
                    [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                    % Step 4: Corrupt pose with noise
                    [obj.ClassOdometryModel,odometryData] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                    % Step 5: Use Particle Filter
                    obj.ClassParticleFilter = obj.ClassParticleFilter.updateParticles(sensorData,odometryData, relocating, particleMap);
                    [estPath(:,i+1),var] = obj.ClassParticleFilter.getPoseEstimate();
                    spread = mean(var);
                    spreads(i) = spread;
                    % Step 6: Update Coverage Map
                    if ~relocating
                        obj.ClassCoverage = obj.ClassCoverage.updateCoverageMap(obj.ClassParticleFilter.Particles,estPath(:,i+1));
                    end
                    % Coverage
                    v = groundTruth(path(:,i), obj.PolyMap, obj.Resolution);
                    vx = v(1);
                    vy = v(2);
                    if ((vx>=1 && vx<=N) && (vy>=1 && vy<=M))
                        ground(vx, vy) = 1;
                    end
                    obj.ClassCoverage.CoverageMap(mapAbs==0) = 0;
                    coverage = sum(obj.ClassCoverage.CoverageMap>=obj.Threshhold)/sum(mapAbs==1);
                    coverages(i) = coverage;
                end
                figure()
                plot(spreads);
                figure()
                plot(coverages);
                results.spreads = spreads;
            elseif mode == 4
                p = T;
                if T >=1
                    p = 1;
                end
                coverage = 0; i = 0;
                % TODO: Add here choices
                obj.ClassParticleFilter = obj.ClassParticleFilter.initializeParticles(path(:,1),3);
                obj.ClassCoverage = obj.ClassCoverage.initializeCoverageMap(obj.EstPolyMap);
                obj.ClassNNCCPP = obj.ClassNNCCPP.initializeNeuralNet(obj.EstPolyMap);
                spread = 0;
                relocating = false;
                while coverage < p
                    i = i + 1;
                    disp(i*obj.Dt)
                    % Step 1: Get sensor measurements
                    sensorData = obj.ClassGrassSensor.measure(path(:,i));
                    % Step 2: Get control input
                    if spread > obj.WallFollow && ~relocating
                        relocating = true; % change to wallfollowing
                        if coverage < 0.5 % half the map not covered
                            half = false;
                            if ((mod(estPath(3,i), 2*pi) > (3*pi)/2) || (mod(estPath(3,i), 2*pi) < pi/2))
                                sensor = 1; % right sensor
                            else
                                sensor = 0; % left sensor
                            end
                        else % half the map already covered
                            half = true;
                            if ((mod(estPath(3,i), 2*pi) > (3*pi)/2) || (mod(estPath(3,i), 2*pi) < pi/2))
                                sensor = 0; % left sensor
                            else
                                sensor = 1; % right sensor
                            end
                        end
                    end
                    if (relocating && spread < obj.WallFollow/3) % change from wallfollowing to NNCCPP
                        idx_x = ceil((estPath(1,i) - obj.PolyMap.XWorldLimits(1)) * obj.Resolution);
                        idx_y = ceil((estPath(2,i) - obj.PolyMap.YWorldLimits(1)) * obj.Resolution);
                        if ((~half && obj.ClassNNCCPP.ExternalInput(idx_x,idx_y) ~= 0) || (half && obj.ClassNNCCPP.ExternalInput(idx_x,idx_y) == 0))
                            relocating = false;
                            obj.ClassWallFollower = WallFollower();
                        end
                    end
                    if relocating % wallfollow
                        [obj.ClassWallFollower,u] = obj.ClassWallFollower.wallFollowing(sensorData, sensor);
                    else % netting
                        orientation = mod(estPath(3,i), 2*pi);
                        % allocate control weight based on orientation
                        if (orientation > 7.5*pi/4 || orientation < 0.5*pi/4) || (orientation > 3.5*pi/4 && orientation < 4.5*pi/4)
                            obj.ClassNNCCPP.C = 100;
                        else
                            obj.ClassNNCCPP.C = 1.5;
                        end
                        if particleMap % use particlemap
                            [obj.ClassNNCCPP,x] = obj.ClassNNCCPP.planStep(estPath(:,i),obj.ClassParticleFilter.CoverageMap);
                        else % use coveragemap
                            [obj.ClassNNCCPP,x] = obj.ClassNNCCPP.planStep(estPath(:,i),obj.ClassCoverage.CoverageMap);
                        end
                        % get movement data
                        [obj.ClassPDController,u] = obj.ClassPDController.pdControl(estPath(1:2,i),[0;0],x,[0;0],estPath(3,i),0);
                    end
                    % Step 3: Move Robot and store positions
                    [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                    % Step 4: Corrupt pose with noise
                    [obj.ClassOdometryModel,odometryData] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                    % Step 5: Use Particle Filter
                    obj.ClassParticleFilter = obj.ClassParticleFilter.updateParticles(sensorData,odometryData, relocating, particleMap);
                    [estPath(:,i+1),var] = obj.ClassParticleFilter.getPoseEstimate();
                    spread = mean(var);
                    spreads(i) = spread;
                    % Step 6: Update Coverage Map
                    if ~relocating
                        obj.ClassCoverage = obj.ClassCoverage.updateCoverageMap(obj.ClassParticleFilter.Particles,estPath(:,i+1));
                    end
                    % Coverage
                    v = groundTruth(path(:,i), obj.PolyMap, obj.Resolution);
                    vx = v(1);
                    vy = v(2);
                    if ((vx>=1 && vx<=N) && (vy>=1 && vy<=M))
                        ground(vx, vy) = 1;
                    end
                    obj.ClassCoverage.CoverageMap(mapAbs==0) = 0;
                    coverage = sum(obj.ClassCoverage.CoverageMap>=obj.Threshhold)/sum(mapAbs==1)
                    disp(coverage);
                    coverages(i) = coverage;
                end
                figure()
                plot(spreads);
                figure()
                plot(coverages);
                results.spreads = spreads;
            elseif mode == 5
                I = round(T/obj.Dt);
                path = zeros(3,I+1);
                estPath = zeros(3,I+1);
                path(:,1) = obj.Pose;
                obj.ClassParticleFilter = obj.ClassParticleFilter.initializeParticles(path(:,1),3);
                for i = 1:1:I
                    disp(i*obj.Dt)
                    % Step 1: Get sensor measurements
                    sensorData = obj.ClassGrassSensor.measure(path(:,i));
                    % Step 2: Get control input
                    [obj.ClassWallFollower,u] = obj.ClassWallFollower.wallFollowing(sensorData, 1);
                    % Step 3: Move Robot and store positions
                    [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                    % Step 4: Corrupt pose with noise
                    [obj.ClassOdometryModel,odometryData] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                    estPath(:,i+1) = obj.ClassOdometryModel.odometryPose(estPath(:,i), true, 1);
                    obj.ClassParticleFilter = obj.ClassParticleFilter.updateParticles(sensorData,odometryData, false, particleMap, path(:,i+1));
                end
                obj.Pose = path(:,end);
                obj.EstPose = estPath(:,end);
            else
                error('Wrong mode chosen!')
            end
            results.path = path;
            results.estPath = estPath;
            results.coverageMap = obj.ClassCoverage.CoverageMap;
            results.particleCoverageMap = obj.ClassParticleFilter.CoverageMap;
            results.neuralActivity = obj.ClassNNCCPP.NeuralActivity;
            results.externalInput = obj.ClassNNCCPP.ExternalInput;
            results.groundTruth = ground;
            results.coverages = coverages;
        end
    end
end