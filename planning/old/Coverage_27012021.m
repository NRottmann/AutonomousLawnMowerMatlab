classdef Coverage
    % A coverage map class based on (1)
    %
    % Date:     21.01.2021
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    %
    % (1): A Probabilistic Approach to High-Confidence Cleaning Guarantees for Low-Cost Cleaning Robots
    
    properties
        % Storage variables
        Map;                % A map of the world
        CoverageMap;        % A matrix which shows the coverage of the area
        ObstacleMap;        % A matrix which shows the obstacles in the area
        PoseID;             % Pose estimate map ids
        TotalCoverage;      % The current total coverage amount
        TotalCells2Cover;   % The total number of cells which have to be covered    
        
        % Parameters
        Resolution;         % The map resolution
    end
    
    methods
        function obj = Coverage()
            % Constructor
            out = get_config('coverageMap');
            obj.Resolution = out.resolution;
            
            % Initialize Pose estimate
            obj.PoseID = [inf inf];
        end
        
        function obj = initializeCoverageMap(obj,map)
            % This method initializes the coverage and obstacle map based
            % on the given map input
            % Syntax:
            %       obj = initializeCoverageMap(Map)
            % Input:
            %   Map:    A map of the environment
            
            % Allocate map
            obj.Map = map;
            
            % Generate obstacle map and specify world location
            obj.ObstacleMap = binaryOccupancyMap(map.XWorldLimits(2) - map.XWorldLimits(1), ...
                                map.YWorldLimits(2) - map.YWorldLimits(1),obj.Resolution);          
            obj.ObstacleMap.GridLocationInWorld(1) = map.XWorldLimits(1);
            obj.ObstacleMap.GridLocationInWorld(2) = map.YWorldLimits(1);
            
            % Place obstacles
            for i=1:1:obj.ObstacleMap.GridSize(1)
                for j=1:1:obj.ObstacleMap.GridSize(2)
                    setOccupancy(obj.ObstacleMap,[i j],~getOccupancy(map,grid2world(obj.ObstacleMap,[i j])),"grid");
                end
            end
            
            % Generate coverage map and specify world location
            obj.CoverageMap = occupancyMap(map.XWorldLimits(2) - map.XWorldLimits(1), ...
                                map.YWorldLimits(2) - map.YWorldLimits(1),obj.Resolution);          
            obj.CoverageMap.GridLocationInWorld(1) = map.XWorldLimits(1);
            obj.CoverageMap.GridLocationInWorld(2) = map.YWorldLimits(1);
            
            % Set all places to uncovered
            obj.TotalCells2Cover = 0;
            for i=1:1:obj.CoverageMap.GridSize(1)
                for j=1:1:obj.CoverageMap.GridSize(2)
                    if (~getOccupancy(obj.ObstacleMap,[i j],"grid"))
                        obj.TotalCells2Cover = obj.TotalCells2Cover + 1;
                    end
                    setOccupancy(obj.CoverageMap,[i j],0,"grid");
                end
            end
            
            obj.TotalCoverage = 0;
        end
        
        function obj = updateCoverageMap(obj,particles,estPose)
            % This method updates the coverage map given the current
            % particle distribution
            % Syntax:
            %       obj = initializeCoverageMap(particles, estPose)
            % Input:
            %   particles:          Particles form the particle filter
            %   estPose:            Current pose estimate
            
            estPoseID = world2grid(obj.CoverageMap,[estPose(1) estPose(2)]);
            
            % Only update if robot moved to next cell
            if (~isequal(estPoseID,obj.PoseID)) 
                
                obj.PoseID = estPoseID;
                
                % Get number of particles
                n = length(particles(1,:));
                
                % Update coveragemap from new particles
                for i=1:1:n
                    ij = world2grid(obj.CoverageMap,[particles(1,i) particles(2,i)]);
                    occVal = getOccupancy(obj.CoverageMap,ij,'grid');
                    diffOccVal = (1/n) * (0.999 - occVal);
                    
                    if (~getOccupancy(obj.ObstacleMap,ij,"grid"))
                        obj.TotalCoverage = obj.TotalCoverage + (diffOccVal/obj.TotalCells2Cover);
                        disp(obj.TotalCoverage)
                    end
                    
                    occVal = occVal + diffOccVal;
                    setOccupancy(obj.CoverageMap,ij,occVal,'grid');     
                end
            end
        end
    end
end