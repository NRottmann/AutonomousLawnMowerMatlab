classdef NNCCPP
    % Neural Network approach for Complete Coverage Path Planning, the
    % approach is based on (1) and (2)
    %
    % Date:     27.01.2021
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    %
    % (1): A Neural Network Approach to Complete Coverage Path Planning
    % (2): A Probabilistic Approach to High-Confidence Cleaning Guarantees for Low-Cost Cleaning Robots
    
    properties
        % Maps
        Map;                % A map of the world (Occupancy Grid)
        CoverageMap;        % A matrix which shows the coverage of the area
        TrueCoverageMap;    % A mattrix representing the true coverage
        ObstacleMap;        % A matrix which shows the obstacles in the area
        AreaSammplingMap;
        
        % Coverage Values
        TotalCoverage;      % The current total coverage amount
        TrueTotalCoverage;  % The current true total coverage
        TotalCells2Cover;   % The total number of cells which have to be covered  
        
        % Pose
        EstPoseID;          % Pose estimate map ids
        
        % Parameters
        Resolution;         % The map resolution
        Divider;            % Divider for the map resolution if area sampling
             
        % Neural Net Matrices
        NeuralActivity;     % Matrix with values from -D (lower bound) and +B (upper bound)
        TargetPosition;     % Current target position
        Filter;             % Matrix [3,3] used to filter the weights for the shunting equation
        Gradient;
        Bmat;
        Dmat;
        
        % Path Planning Parameter
        A;                  % Passive decay rate
        B;                  % Upper bound
        D;                  % Lower bound
        E;                  % Gain
        C;                  % Control gain  
        Threshhold;         % Threshhold for the coverageMap
        G;                  % Descent of the gradient

        Dt;                 % Step time
    end
    
    methods
        function obj = NNCCPP()
            % Constructor
            out = get_config('coverageMap');
            obj.Resolution = out.resolution;
            
            % Initialize Pose estimate
            obj.EstPoseID = [inf inf];
            
            % Constructor
            out = get_config('planning');
            obj.A = out.a;
            obj.B = out.b;
            obj.D = out.d;
            obj.E = out.e;
            obj.C = out.c;
            obj.Threshhold = out.threshhold;
            obj.Dt = out.dt;
            obj.G = out.g;
            
            out = get_config('coverageMap');
            obj.Resolution = out.resolution;
            obj.Divider = out.divider;
            
            % The filter representing the weights
            obj.Filter = [1/sqrt(2) 1 1/sqrt(2); 1 0 1; 1/sqrt(2) 1 1/sqrt(2)];
            
        end
        
        function obj = initializeNNCCPP(obj,map)
            % This method initializes the coverage and obstacle map based
            % on the given map input
            % Syntax:
            %       obj = initializeCoverageMap(Map)
            % Input:
            %   map:    A map of the environment
            
            % Generate map according to desired resolution
            obj.Map = binaryOccupancyMap(map.XWorldLimits(2) - map.XWorldLimits(1), ...
                                map.YWorldLimits(2) - map.YWorldLimits(1),obj.Resolution);          
            obj.Map.GridLocationInWorld(1) = map.XWorldLimits(1);
            obj.Map.GridLocationInWorld(2) = map.YWorldLimits(1);
            % Set map information
            for i=1:1:obj.Map.GridSize(1)
                for j=1:1:obj.Map.GridSize(2)
                    setOccupancy(obj.Map,[i j],getOccupancy(map,grid2world(obj.Map,[i j])),"grid");
                end
            end
            
            % Generate obstacle map
            obj.ObstacleMap = zeros(obj.Map.GridSize);
            % Place obstacles
            for i=1:1:obj.Map.GridSize(1)
                for j=1:1:obj.Map.GridSize(2)
                    obj.ObstacleMap(i,j) = 1 - getOccupancy(obj.Map,[i j],"grid");
                end
            end
            
            % Generate coverage map
            obj.CoverageMap = zeros(obj.Map.GridSize);
            % Count cells
            obj.TotalCells2Cover = 0;
            for i=1:1:obj.Map.GridSize(1)
                for j=1:1:obj.Map.GridSize(2)
                    if (obj.ObstacleMap(i,j) == 0)
                        obj.TotalCells2Cover = obj.TotalCells2Cover + 1;
                    end
                end
            end
            
            % Generate area sampling map
            obj.AreaSammplingMap = binaryOccupancyMap(map.XWorldLimits(2) - map.XWorldLimits(1), ...
                                map.YWorldLimits(2) - map.YWorldLimits(1),obj.Resolution*obj.Divider);          
            obj.AreaSammplingMap.GridLocationInWorld(1) = map.XWorldLimits(1);
            obj.AreaSammplingMap.GridLocationInWorld(2) = map.YWorldLimits(1);
            % Set map information
            for i=1:1:obj.AreaSammplingMap.GridSize(1)
                for j=1:1:obj.AreaSammplingMap.GridSize(2)
                    setOccupancy(obj.AreaSammplingMap,[i j],getOccupancy(map,grid2world(obj.AreaSammplingMap,[i j])),"grid");
                end
            end
            
            % Generate true coverage map
            obj.TrueCoverageMap = zeros(obj.Map.GridSize);
            
            % Set total coverage to zero
            obj.TotalCoverage = 0;
            obj.TrueTotalCoverage = 0;
            
            % Iniitialize Neural Net matrices
            obj.NeuralActivity = zeros(obj.Map.GridSize);
            obj.Bmat = obj.B*ones(obj.Map.GridSize);
            obj.Dmat = obj.D*ones(obj.Map.GridSize);
            
            % Initialize current target position
            obj.TargetPosition = zeros(2,1);

            % Gradient
            gradient = linspace(1,obj.G,obj.Map.GridSize(1))';
            obj.Gradient = repmat(gradient,1,obj.Map.GridSize(2));
        end
     
        function obj = updateCoverageMap(obj,particles,estPose)
            % This method updates the coverage map given the current
            % particle distribution
            % Syntax:
            %       obj = initializeCoverageMap(particles, estPose)
            % Input:
            %   particles:          Particles form the particle filter
            %   estPose:            Current pose estimate
            
            estPoseID = world2grid(obj.Map,[estPose(1) estPose(2)]);
            
            % Only update if robot moved to next cell
            if (~isequal(estPoseID,obj.EstPoseID)) 
                
                obj.EstPoseID = estPoseID;
                
                % Get number of particles
                n = length(particles(1,:));
                
                % Update coveragemap from new particles
                for i=1:1:n
                    ij = world2grid(obj.Map,[particles(1,i) particles(2,i)]);
                    if ((ij(1) >= 1 && ij(1) <= obj.Map.GridSize(1)) && ...
                            (ij(2) >= 1 && ij(2) <= obj.Map.GridSize(2)))
                        occVal = obj.CoverageMap(ij(1),ij(2));
                        diffOccVal = (1/n) * (1 - occVal);

                        if (obj.ObstacleMap(ij(1),ij(2)) == 0)
                            obj.TotalCoverage = obj.TotalCoverage + (diffOccVal/obj.TotalCells2Cover);
                        end

                        occVal = occVal + diffOccVal;
                        obj.CoverageMap(ij(1),ij(2)) = occVal;
                    end
                end
            end
        end
        
        function obj = updateTrueCoverageMap(obj,pose)
            % This method updates the coverage map given the current
            % particle distribution
            % Syntax:
            %       obj = initializeCoverageMap(particles, estPose)
            % Input:
            %   particles:          Particles form the particle filter
            %   estPose:            Current pose estimate
            
            ij = world2grid(obj.Map,[pose(1) pose(2)]);
            occVal = obj.TrueCoverageMap(ij(1),ij(2));
            diffOccVal = (1 - occVal);
            if (obj.ObstacleMap(ij(1),ij(2)) == 0)
                obj.TrueTotalCoverage = obj.TrueTotalCoverage + (diffOccVal/obj.TotalCells2Cover);
            end
            occVal = occVal + diffOccVal;
            obj.TrueCoverageMap(ij(1),ij(2)) = occVal;
            
        end
           
        function [obj] = updateNeuralActivity(obj,tmpCoverageMap)
            % Update the neural activity of the neurons based on the
            % shunting equation
            
            % Determine neural activity and positive neural activity
            Xp = obj.NeuralActivity;                        
            Xp(Xp < 0) = 0;

            % Generate weighted inhibitory inputs for the shunting equation
            Xp_tmp = imfilter(Xp, obj.Filter);   
            Xp = Xp_tmp;

            
            % Generate external inputs based on obstacles and coverage
            Ip = obj.E * (1 - (tmpCoverageMap + obj.ObstacleMap));
            In = obj.E * obj.ObstacleMap;                     

            % calculate the change for the neural activity
            dX = -obj.A*obj.NeuralActivity + (obj.Bmat - obj.NeuralActivity) ...
                .* (Ip + Xp) - (obj.Dmat + obj.NeuralActivity) .* In;
            
            % Update the neural activity
            obj.NeuralActivity = obj.NeuralActivity + dX*obj.Dt;
        end
        
        function [obj,x] = planStep(obj,pose)
            % Plan the next step
            %
            % Input:
            %   pose:           Current pose of the robot
            %   coverageMap:    Current coverage map for the robot
            %
            % Output:
            %   obj:        Instance of the class
            %   x:          Next target position

            % Get the index according to the position of the vehicle
            ij = world2grid(obj.Map,[pose(1) pose(2)]);

            % Assume current pose is covered
            tmpCoverageMap = obj.CoverageMap;
            tmpCoverageMap(ij(1),ij(2)) = 1;
            
            % Update neural activity
            obj = updateNeuralActivity(obj,tmpCoverageMap);

            % Plan the next step
            obj = planning(obj,pose);
            x = obj.TargetPosition;
        end
        
    	function obj = planning(obj,pose)
            % Choose the position to go next
            %
            % Input:
            %   obj:    Instance of the class
            %   pose:   Current pose estimate
   
            % Get the index according to the position of the vehicle
            ij = world2grid(obj.Map,[pose(1) pose(2)]);
            
            % Get index boundaries
            N = obj.Map.GridSize(1);
            M = obj.Map.GridSize(2);

            % Adjusted Neural Activity
            X = obj.NeuralActivity .* obj.Gradient;
            
            % Go through all neighbours and decide where to go next
            dec = -inf;
            for i=-1:1:1
                for j=-1:1:1
                    % Check if out of bounds
                    if ((i+ij(1)>=1 && i+ij(1)<=N) && (j+ij(2)>=1 && j+ij(2)<=M)) 
                        % Do if value of cell is higher then initialPose and the cell is not the initial cell
                        if (X(ij(1),ij(2)) <= X(ij(1)+i,ij(2)+j) && ~(i==0 && j==0))      
                            ii = ij(1) + i;
                            jj = ij(2) + j;
                            % Calculate orientation difference for control input
                            currentOrientation = mod(pose(3), 2*pi);
                            movementVector = grid2world(obj.Map,[ii jj]) - grid2world(obj.Map,[ij(1) ij(2)]);
                            movementOrientation = atan2(movementVector(2),movementVector(1));
                            orientationDiff = abs(angdiff(currentOrientation,movementOrientation));
                            % Calculate weighted control input
                            dec_tmp  = X(ii,jj) - obj.C * orientationDiff;   
                            % Update control input
                            if dec_tmp > dec
                                obj.TargetPosition = grid2world(obj.Map,[ii jj])';
                                dec = dec_tmp;
                            end
                        end
                    end
                end
            end
        end
        
        function obj = areaSampling(obj,pose,pastPose)
            % Function for generating an area sampling approach. Therefore,
            % we assume that the swiped area of a pixel is a rectangular.
            % We first enlarge the resolution of the grid by the factor F
            % and then check which cells are occupied. Then we reduce the
            % resolution back to the actual one.
            
            v = pose(1:2) - pastPose(1:2);
            
            
        end
    end
end