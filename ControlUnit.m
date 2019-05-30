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
        PolyMap;
        EstPolyMap;
        Pose;
        EstPose;
        
        %% Parameters
        Dt;
    end
    
    methods
        function obj = ControlUnit(polyMap,pose)
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
            obj.ClassGrassSensor = GrassSensor(polyMap);
            
            % Controller
            obj.ClassWallFollower = WallFollower();
            obj.ClassRandomController = RandomController();
            obj.ClassPDController = PDController();
            
            % Mapping
            obj.ClassPoseGraphOptimization = PoseGraphOptimization();
            obj.ClassMapPostProcessing = MapPostProcessing([polyMap.x; polyMap.y],0);
            
            % Localization
            obj.ClassGlobalLocalizer = GlobalLocalizer(polyMap);
            
            % Planning
            obj.ClassCoverage = Coverage();
            obj.ClassNNCCPP = NNCCPP();
            
            %% Initialize variables
            obj.PolyMap = polyMap;
            obj.EstPolyMap = polyMap;
            obj.Pose = pose;
            obj.EstPose = pose;
            
            %% Get Parameters
            out = get_config('system');
            obj.Dt =  out.dt;
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
                path(:,1) = generateStartPose(obj.PolyMap);     % Start with random initial start pose
            elseif mode == 1
                path(:,1) = obj.Pose;
            else
                error('Wrong mode chosen!')
            end
            for i = 1:1:I
                % Step 1: Get sensor measurements
                sensorData = obj.ClassGrassSensor.measure(path(:,i));
                % Step 2: Get control input
                [obj.ClassWallFollower,u] = obj.ClassWallFollower.wallFollowing(sensorData);
                % Step 3: Move Robot and store positions
                [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                % Step 4: Corrupt pose with noise
                [obj.ClassOdometryModel,~] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                estPath(:,i+1) = obj.ClassOdometryModel.odometryPose(estPath(:,i), true, 1);
            end
            obj.Pose = path(:,end);
            obj.EstPose = estPath(:,end);
        end
        
        function [obj,results] = mapping(obj,path,optimize)
            % This is the mapping mode
            % Syntax:
            %       results = mapping(obj,path)
            % Input:
            %   path:           Path data used for generating the map
            %   optimize:   struct for optimization
            %       loopClosure:    if true, we optimize l_nh and c_min
            %       mapping:        if true, we optimize gamma1 and gamma2
            % Output:   
            %   results:        Results of the mapping approach
            %       optimizedPath:  The whole path after optimization
            %       cuttedPath:     path with cutted edges
            %       closedPath:     Closed path
            %       polyMap:        The map estimate
            
            % Generate optimized path data
            [obj.ClassPoseGraphOptimization,path,A,Circumference] = obj.ClassPoseGraphOptimization.generateMap(path(1:2,:),optimize);
            obj.ClassMapPostProcessing.DP = path(1:2,:);
            obj.ClassMapPostProcessing.A = A;
            obj.ClassMapPostProcessing.Circumference = Circumference;
            % Cut ends
            obj.ClassMapPostProcessing = obj.ClassMapPostProcessing.cutGraph();
            % Close graph
            obj.ClassMapPostProcessing = obj.ClassMapPostProcessing.closeGraph();
            % Generate poly map from closed graph
            obj.ClassMapPostProcessing = obj.ClassMapPostProcessing.generatePolyMap();
            % Adjust estimated map
            obj.EstPolyMap = obj.ClassMapPostProcessing.EstMap;
            % Allocate results
            results.estMap = obj.ClassMapPostProcessing.EstMap;      
            results.DP = obj.ClassMapPostProcessing.DP;             
            results.cutDP = obj.ClassMapPostProcessing.CutDP;          
            results.closedDP = obj.ClassMapPostProcessing.ClosedDP;       
            results.A = obj.ClassMapPostProcessing.A;             
            results.cutA = obj.ClassMapPostProcessing.CutA;
            results.param.gamma(1,1) = obj.ClassPoseGraphOptimization.Gamma1;
            results.param.gamma(1,2) = obj.ClassPoseGraphOptimization.Gamma2;
            results.param.alpha(1,1) = obj.ClassPoseGraphOptimization.Alpha1;
            results.param.alpha(1,2) = obj.ClassPoseGraphOptimization.Alpha2;
            results.param.alpha(1,3) = obj.ClassPoseGraphOptimization.Alpha3;
            results.param.alpha(1,4) = obj.ClassPoseGraphOptimization.Alpha4;
            results.param.l_min = obj.ClassPoseGraphOptimization.L_min;
            results.param.e_max = obj.ClassPoseGraphOptimization.E_max;
            results.param.l_nh = obj.ClassPoseGraphOptimization.L_nh;
            results.param.c_min = obj.ClassPoseGraphOptimization.C_min;
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
        
        function [obj,results] = completeCoverage(obj,T,mode)
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
            estPath = zeros(3,I+1);
            
            % Decide which mode
            if mode == 1
                % Initialize
                odometryData.DeltaR1 = 0; odometryData.DeltaT = 0; odometryData.DeltaR2 = 0;
                % TODO: Add here choices
                obj.ClassParticleFilter = obj.ClassParticleFilter.initializeParticles(path(:,1),3);
                obj.ClassCoverage = obj.ClassCoverage.initializeCoverageMap(obj.EstPolyMap);
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
                    obj.ClassParticleFilter = obj.ClassParticleFilter.updateParticles(sensorData,odometryData);
                    [estPath(:,i+1),~] = obj.ClassParticleFilter.getPoseEstimate();
                    % Step 6: Update Coverage Map
                    obj.ClassCoverage = obj.ClassCoverage.updateCoverageMap(obj.ClassParticleFilter.Particles,estPath(:,i+1));
                end
            elseif mode == 2
                % TODO: Add here choices
                obj.ClassParticleFilter = obj.ClassParticleFilter.initializeParticles(path(:,1),3);
                obj.ClassCoverage = obj.ClassCoverage.initializeCoverageMap(obj.EstPolyMap);
                obj.ClassNNCCPP = obj.ClassNNCCPP.initializeNeuralNet(obj.EstPolyMap);
                for i=1:1:I
                    disp(i*obj.Dt)
                    % Step 1: Get sensor measurements
                    sensorData = obj.ClassGrassSensor.measure(path(:,i));
                    % Step 2: Get control input
                    [obj.ClassNNCCPP,x] = obj.ClassNNCCPP.planStep(estPath(:,i),obj.ClassParticleFilter.CoverageMap);
                    [obj.ClassPDController,u] = obj.ClassPDController.pdControl(estPath(1:2,i),[0;0],x,[0;0],estPath(3,i),0);
                    % Step 3: Move Robot and store positions
                    [path(:,i+1),motionData] = obj.ClassKinematicModel.kinModel(path(:,i), u, true);
                    % Step 4: Corrupt pose with noise
                    [obj.ClassOdometryModel,odometryData] = obj.ClassOdometryModel.odometryData(path(:,i), motionData);
                    % Step 5: Use Particle Filter
                    obj.ClassParticleFilter = obj.ClassParticleFilter.updateParticles(sensorData,odometryData);
                    [estPath(:,i+1),~] = obj.ClassParticleFilter.getPoseEstimate();
                    % Step 6: Update Coverage Map
                    obj.ClassCoverage = obj.ClassCoverage.updateCoverageMap(obj.ClassParticleFilter.Particles,estPath(:,i+1));
                end
            else
                error('Wrong mode chosen!')
            end
            results.path = path;
            results.estPath = estPath;
            results.coverageMap = obj.ClassCoverage.CoverageMap;
            results.particleCoverageMaps = obj.ClassParticleFilter.ParticleCoverageMaps;
            results.particleCoverageMap = obj.ClassParticleFilter.CoverageMap;
        end
    end    
end