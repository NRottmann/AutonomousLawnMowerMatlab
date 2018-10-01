classdef ParticleFilter
    % Particle Filter class for autonomous lawn mower
    % Methods:
    %   ParticleFilter(numParticles,polyMap,p0)
    %   obj = update(obj,odometryData,sensorData)
    %   posParticles = getParticlePosition(obj)
    %   posParticles = getParticlePositionOrientationWeight(obj)
%     %   obj = updateParticleProbabiltyMaps(obj)
%     %   probMap = getParticleProbabilityMaps(obj)
    %   rating = getParticleRating(obj)
    %   [mu,sigma] = getMeanVariance(obj)
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 01.10.2018
    
    properties
        NumParticles;                   % Array of min and max Particles
        ActParticles;                   % Actual number of particles used
        Particles;                      % Array of particles with every
                                        % row is one particle and contains
                                        % [x; y; phi; weight]
        PolyMap;
% 
%         ParticleProbabilityMaps;        % Cell array for probability maps
%                                         % for each particle
%         ProbabilityMap;                 % Fused Probability Map
                                        
    end
    
    methods
        function obj = ParticleFilter(numParticles,polyMap,p0)
            % This is the constructor of the class
            % Syntax:
            %       obj = ParticleFilter(nParticles,polyMap,p0)
            % Input:
            %   numParticles:   The number of particles as [min; max]
            %   polyMap:     	The map as polygon
            %   p0:             A start Pose for the particles with 
            %                   p = [x,y,phi,known]^T, if known = 0, the
            %                   starting poses will be chosen randomly
            % Output:
            %   obj:            Intance of the class ParticleFilter
            
            % Check for correct dimensions
            if (size(numParticles) ~= [2 1])
                error('Size of input nParticles is not correct!')
            end
            
            % Allocate Parameters
            obj.NumParticles = numParticles;
            obj.PolyMap = polyMap;
            
            % Initialize Particles
            obj.Particles = zeros(4, numParticles(2));
            if p0(4) == 1
                obj.ActParticles = obj.NumParticles(1);
                for i = 1:1:obj.ActParticles
                    obj.Particles(:,i) = [p0(1); p0(2); ...
                                        p0(3); 1/obj.ActParticles];
                end
            else
                obj.ActParticles = obj.NumParticles(2);
                dx = polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1);
                dy = polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1);
                for i = 1:1:obj.ActParticles
                    obj.Particles(:,i) = [rand()*dx + polyMap.XWorldLimits(1); ...
                                          rand()*dy + polyMap.YWorldLimits(1); ...
                                          rand()*2*pi; 1/obj.ActParticles];
                end   
            end
            
            % TODO: Add method to generate sufficient accuarte occupancy
            % grid maps based on the polyMap
%             % Generate Probability Maps for the max number of Particles
%             obj.ParticleProbabilityMaps = cell(numberParticles(2),1);
%             for i=1:1:numberParticles(2)
%                 obj.ParticleProbabilityMaps{i} = zeros(map.GridSize);
%             end
%             
%             % Generate Probability Map
%             obj.ProbabilityMap = zeros(map.GridSize);
        end
        
        function obj = update(obj,odometryData,sensorData)
            % This is the update function of the class which holds the
            % particle filter algorithm
            % Syntax:
            %       obj = update(obj,odometryData,sensorData)
            % Input:
            %   obj:            Object of the Particle Filter class
            %   odometryData:   deltaR1, deltaT, deltaR2
            %   sensorData:     data from the sensor estimation
            % Output:
            %   obj:            Instance of the class ParticleFilter
            
            % For all Particles, update them according to odometryData and
            % allocate weights according to measurement data
            for i = 1:1:obj.ActParticles
                % TODO: Is the check required? Check if Particle is 
                % actually used
                if obj.Particles(4, i) == -1
                    continue;
                end
                
                % Move Particle, add noise
                obj.Particles(1:3,i) = odometryPose(obj.Particles(1:3,i),...
                                            odometryData, true);
                                        
                % Allocate Weights, therefore check what the sensor should
                % measure depending on the particles position and compare
                % this with the actual measurement
                out = get_config('particleFilter');
                weightFactors =  out.weightFactors;
                [sensorParticleData] = grassSensor(obj.Particles(1:3,i),obj.PolyMap);     
                if ((sensorParticleData.right == sensorData.right) && (sensorParticleData.left == sensorData.left))
                    obj.Particles(4,i) = weightFactors(1);
                elseif ((sensorParticleData.right == sensorData.right) || (sensorParticleData.left == sensorData.left))
                    obj.Particles(4,i) = weightFactors(2);
                else
                    obj.Particles(4,i) = weightFactors(3);    
                end 
            end
            
            % Now we are doing the resampling of the particles based on the
            % calculated weights. Therefore we first normalize the weights
            % and calculate the effective number of particles
            obj.Particles(4,:) = obj.Particles(4,:) / sum(obj.Particles(4,:));
            N_eff = 1 / sum(obj.Particles(4,:).^2);

            % Do only resampling if N_eff < a*N
            out = get_config('particleFilter');
            treshResample =  out.treshResample;
            if (N_eff < treshResample * obj.ActParticles)
                
                % Check how good the Particle Filter estimates the position 
                % and change the particle Size according to this
                % TODO: Check if this strategy is valid
                rating = obj.getParticleRating();
                scale = obj.NumParticles(2) - obj.NumParticles(1);
                scale = scale * (1-rating);
                if (1-rating) < 10^(-2) % handle division by 0
                    scale = obj.NumParticles(1);
                else
                    scale = obj.NumParticles(1) + ceil(scale);
                end
                newCalculatedParticles = scale;
                
                % Create Weighting vector
                weightVec = zeros(obj.ActParticles+1,1);
                for i=1:1:obj.ActParticles
                    weightVec(i+1) = weightVec(i) + obj.Particles(4,i);
                end
                % Random resampling
                tempParticles = zeros(4,obj.NumParticles(2));
%                 tempProbMap = cell(obj.NumParticles(2),1);
                for i=1:1:newCalculatedParticles
                    randNum = rand();
                    k = 1;
                    while (randNum > weightVec(k+1))
                        k = k + 1;
                    end
                    tempParticles(:,i) = obj.Particles(:,k);
%                     tempProbMap{i} = obj.ParticleProbabilityMaps{k};
                end
                % Allocate Particles and set new NumParticles
                obj.Particles = tempParticles;
%                 obj.ParticleProbabilityMaps = tempProbMap;
                obj.ActParticles = newCalculatedParticles;
            end            
        end
             
       % TODO: Check which functions are really required
        function posParticles = getParticlePosition(obj)
            % Function to give back the positions of the particles
            posParticles = obj.Particles(1:2,1:obj.ActParticles);
        end
        
        function posParticles = getParticlePositionOrientationWeight(obj)
            % Function to get the positions of the particles with the 
            % weight and the orientation 
            posParticles = obj.Particles(:,1:obj.ActParticles);
        end
%         
%         function obj = updateParticleProbabiltyMaps(obj)
%             % Function to update the Probability Map of the particle Filter
%             for i=1:1:obj.ActParticles
%                 if (obj.Particles(1,i) > obj.PolyMap.XWorldLimits(1) ...
%                     && obj.Particles(1,i) < obj.PolyMap.XWorldLimits(2) ...
%                     && obj.Particles(2,i) > obj.PolyMap.YWorldLimits(1) ...
%                     && obj.Particles(2,i) < obj.PolyMap.YWorldLimits(2) ...
%                     && getOccupancy(obj.PolyMap,[obj.Particles(1,i) obj.Particles(2,i)]))
%                     ind_x = ceil(obj.Particles(1,i)*obj.Map.Resolution);
%                     ind_y = ceil(obj.Particles(2,i)*obj.Map.Resolution);
%                     obj.ParticleProbabilityMaps{i}(ind_x,ind_y) = 1;
%                 end
%             end
%         end
        
%         function obj = updateProbabilityMap(obj)
%             % This function updates a combined probability map. Make sure
%             % you call that function properly in order to avoid a too high
%             % or low update rate
%             probMap = zeros(obj.Map.GridSize);
%             for i=1:1:obj.CalculatedParticles
%                 if (obj.Particles(1,i) > obj.Map.XWorldLimits(1) ...
%                     && obj.Particles(1,i) < obj.Map.XWorldLimits(2) ...
%                     && obj.Particles(2,i) > obj.Map.YWorldLimits(1) ...
%                     && obj.Particles(2,i) < obj.Map.YWorldLimits(2) ...
%                     && getOccupancy(obj.Map,[obj.Particles(1,i) obj.Particles(2,i)]))
%                     ind_x = ceil(obj.Particles(1,i)*obj.Map.Resolution);
%                     ind_y = ceil(obj.Particles(2,i)*obj.Map.Resolution);
%                     probMap(ind_x,ind_y) = probMap(ind_x,ind_y) + (1/obj.CalculatedParticles);
%                 end
%             end
%             obj.ProbabilityMap = obj.ProbabilityMap + probMap - obj.ProbabilityMap .* probMap;  
%         end
        
%         function probMap = getParticleProbabilityMaps(obj)
%             % Gives back the probability map based on each idivdual 
%             % probability map for each particle
%             probMap = zeros(obj.Map.GridSize);
%             for i=1:1:obj.CalculatedParticles
%                 probMap = probMap + obj.ParticleProbabilityMaps{i};
%             end
%             probMap = probMap / obj.CalculatedParticles;
%         end
        
        function rating = getParticleRating(obj)
            % Compute the likelihood of the position
            
            % Get the positions and the max and mins
            positions = obj.getParticlePositionOrientationWeight();
            [~,minX] = min(positions(1,:));
            minX = abs(positions(1,minX));
            [~,maxX] = max(positions(1,:));
            maxX = abs(positions(1,maxX));
            [~,minY] = min(positions(2,:));
            minY = abs(positions(2,minY));
            [~,maxY] = max(positions(2,:));
            maxY = abs(positions(2,maxY));
            
            % Calculate the square of the pf
            bottom = maxX - minX;
            left = maxY - minY;
            sizeOf = bottom*left;
            maxMapSize = obj.PolyMap.XWorldLimits(2) * obj.PolyMap.YWorldLimits(2);
            rating = 1 - (sizeOf/maxMapSize);
            if rating > 1
                rating = 1;
            end
        end
        
        function [mu,sigma] = getMeanVariance(obj)
            mu = mean(obj.Particles(1:3,1:obj.ActParticles),2)';
            sigma = var(obj.Particles(1:3,1:obj.ActParticles),2)';
        end
    end
end

